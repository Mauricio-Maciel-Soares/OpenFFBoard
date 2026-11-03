/*
 * HidFFB.cpp
 *
 *  Created on: 12.02.2020
 *      Author: Yannick
 */

#include <assert.h>
#include "HidFFB.h"
#include "flash_helpers.h"
#include "hid_device.h"
#include "cppmain.h"
#include <math.h>

HidFFB::HidFFB(std::shared_ptr<EffectsCalculator> ec,uint8_t axisCount) : effects_calc(ec), effects(ec->effects),axisCount(axisCount)
{
	directionEnableMask = 1 << axisCount; // Direction enable bit is last bit after axis enable bits
	// Initialize reports
	blockLoad_report.effectBlockIndex = 1;
	updatePoolAvailable();
	blockLoad_report.loadStatus = 1;

	pool_report.ramPoolSize = effects.size()*sizeof(FFB_Effect);
	pool_report.maxSimultaneousEffects = effects.size();
	pool_report.memoryManagement = 1;


	this->registerHidCallback();
}

HidFFB::~HidFFB() {
}


/**
 * Sets the mask where the direction enable bit is in the effect
 */
void HidFFB::setDirectionEnableMask(uint8_t mask){
	this->directionEnableMask = mask;
}

void HidFFB::updateSamplerate(float newSamplerate){
	effects_calc->updateSamplerate(newSamplerate);
}


bool HidFFB::getFfbActive(){
	return this->ffb_active;
}

bool HidFFB::HID_SendReport(uint8_t *report,uint16_t len){
	return tud_hid_report(0, report, len); // ID 0 skips ID field
}


/**
 * Refreshes the aggregate state exposed by this firmware's PID descriptor
 */
void HidFFB::updateStatusReport(){
	// Power is the existing logical PID power indication, not motor telemetry.
	reportFFBStatus.status = HID_ACTUATOR_POWER;
	if(paused) reportFFBStatus.status |= HID_EFFECT_PAUSE;
	if(!hostDisabled && (ffb_active || paused)) reportFFBStatus.status |= HID_ENABLE_ACTUATORS;
	for(const auto& effect : effects){
		if(effect.type != FFB_EFFECT_NONE && effect.state == 1){
			reportFFBStatus.status |= HID_EFFECT_PLAYING;
			break;
		}
	}
}

void HidFFB::sendStatusReport(uint8_t /*effect*/){
	updateStatusReport();
	HID_SendReport(reinterpret_cast<uint8_t*>(&reportFFBStatus), sizeof(reportFFB_status_t));
}

/**
 * Called when HID OUT data is received via USB
 */
void HidFFB::hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize){
	if(buffer == nullptr) return;
	const uint8_t id = report_id - FFB_ID_OFFSET;
	if(id == HID_ID_NEWEFREP){
		if(report_type != HID_REPORT_TYPE_FEATURE || bufsize < sizeof(FFB_CreateNewEffect_Feature_Data_t)) return;
	}else{
		if(report_type != HID_REPORT_TYPE_OUTPUT && report_type != HID_REPORT_TYPE_INVALID) return;
		uint16_t required = 0;
		switch(id){
		case HID_ID_EFFREP: required = 14 + 4 * axisCount; break; // direction + type-specific offset per axis
		case HID_ID_CTRLREP: case HID_ID_GAINREP: case HID_ID_BLKFRREP: required = 2; break;
		case HID_ID_ENVREP: required = sizeof(FFB_SetEnvelope_Data_t); break;
		case HID_ID_CONDREP: required = sizeof(FFB_SetCondition_Data_t); break;
		case HID_ID_PRIDREP: required = sizeof(FFB_SetPeriodic_Data_t); break;
		case HID_ID_CONSTREP: required = sizeof(FFB_SetConstantForce_Data_t); break;
		case HID_ID_RAMPREP: required = sizeof(FFB_SetRamp_Data_t); break;
		case HID_ID_EFOPREP: required = sizeof(FFB_EffOp_Data_t); break;
		default: return;
		}
		if(bufsize < required || buffer[0] != report_id) return;
	}
	fxUpdateEvent(); // use uint16_t for timer overflow handling if micros timer is used

	// FFB Output Message
	const uint8_t* report = buffer;
	uint8_t event_idx = report_id - FFB_ID_OFFSET;

	// -------- Out Reports --------
	switch(event_idx)
	{

		case HID_ID_NEWEFREP: //add Effect Report. Feature
			new_effect((FFB_CreateNewEffect_Feature_Data_t*)(report));
			break;
		case HID_ID_EFFREP: // Set Effect
		{
			FFB_SetEffect_t setEffectRepBuf{};
			memcpy(&setEffectRepBuf,report,std::min<uint16_t>(sizeof(FFB_SetEffect_t),bufsize)); // Copy report to buffer. only valid range if less axes are used
			set_effect(&setEffectRepBuf);
			break;
		}
		case HID_ID_CTRLREP: // Control report. 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
			ffb_control(report[1]);
			//sendStatusReport(0);
			break;
		case HID_ID_GAINREP: // Set global gain
			set_gain(report[1]);
			break;
		case HID_ID_ENVREP: // Envelope
			set_envelope((FFB_SetEnvelope_Data_t *)report);
			break;
		case HID_ID_CONDREP: // Spring, Damper, Friction, Inertia
			set_condition((FFB_SetCondition_Data_t*)report);
			break;
		case HID_ID_PRIDREP: // Periodic
			set_periodic((FFB_SetPeriodic_Data_t*)report);
			break;
		case HID_ID_CONSTREP: // Constant
			set_constant_effect((FFB_SetConstantForce_Data_t*)report);
			break;
		case HID_ID_RAMPREP: // Ramp
			set_ramp((FFB_SetRamp_Data_t *)report);
			break;
		case HID_ID_CSTMREP: // Custom. pretty much never used
			//printf("Customrep");
			break;
		case HID_ID_SMPLREP: // Download sample
			//printf("Sampledl");
			break;
		case HID_ID_EFOPREP: //Effect operation
		{
			set_effect_operation((FFB_EffOp_Data_t*)report);
			break;
		}
		case HID_ID_BLKFRREP: // Free a block
		{
			free_effect(report[1]);
			break;
		}

		default:
		{
			break;
		}
	}

}


/**
 * Called on HID feature GET events
 * Any reply is assigned to the return buffer
 *
 * Handles block load reports and pool status which are requested after a new effect has been created
 */
uint16_t HidFFB::hidGet(uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen){
	if(buffer == nullptr || reqlen == 0) return 0;
	const uint8_t id = report_id - FFB_ID_OFFSET;
	const void* source = nullptr;
	uint16_t size = 0;
	if(report_type == HID_REPORT_TYPE_INPUT && id == HID_ID_STATE){
		updateStatusReport();
		source = &reportFFBStatus.status; // TinyUSB supplies the report ID
		size = sizeof(reportFFBStatus.status);
	}else if(report_type == HID_REPORT_TYPE_FEATURE){
		if(id == HID_ID_BLKLDREP){
			updatePoolAvailable();
			source = &blockLoad_report;
			size = sizeof(blockLoad_report);
		}else if(id == HID_ID_POOLREP){
			source = &pool_report;
			size = sizeof(pool_report);
		}
	}
	if(source == nullptr) return 0;
	size = std::min(size, reqlen);
	memcpy(buffer, source, size);
	return size;
}

void HidFFB::start_FFB(){
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("FFB on");
#endif
	this->set_FFB(true);
}

void HidFFB::stop_FFB(){
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("FFB off");
#endif
	this->set_FFB(false);
}

void HidFFB::set_FFB(bool state)
{
	assert(effects_calc != nullptr);
	this->ffb_active = state && !hostDisabled && !paused;
	effects_calc->setActive(this->ffb_active);
}

void HidFFB::set_gain(uint8_t gain){
	assert(effects_calc != nullptr);
	effects_calc->setGain(gain);
}

void HidFFB::set_filters(FFB_Effect *effect){
	assert(effects_calc != nullptr);
	effects_calc->setFilters(effect);
}

void HidFFB::ffb_control(uint8_t cmd){
	// This firmware's descriptor declares six independent control bits.
	if(cmd & 0x01){
		hostDisabled = false;
		start_FFB();
	}
	if(cmd & 0x02){
		hostDisabled = true;
		stop_FFB();
	}
	if(cmd & 0x04){ // Stop effects without disabling the actuators
		for(auto& effect : effects) effect.state = 0;
	}
	if(cmd & 0x08){
		stop_FFB();
		reset_ffb();
		start_FFB();
	}
	if((cmd & 0x10) && !paused){
		pausedAt = HAL_GetTick();
		paused = true;
		stop_FFB();
	}
	if(cmd & 0x20){
		if(paused){
			const uint32_t elapsed = HAL_GetTick() - pausedAt;
			for(auto& effect : effects){
				if(effect.state == 1) effect.startTime += elapsed;
			}
			paused = false;
		}
		start_FFB(); // Does not override an explicit Disable Actuators
	}
	sendStatusReport(0);
}

void HidFFB::set_constant_effect(FFB_SetConstantForce_Data_t* data){
	if(data->effectBlockIndex == 0 || data->effectBlockIndex > effects.size()){
		return;
	}
	cfUpdateEvent();
	FFB_Effect& effect_p = effects[data->effectBlockIndex-1];

	effect_p.magnitude = data->magnitude;
//	if(effect_p.state == 0){
//		effect_p.state = 1; // Force start effect
//	}
}

void HidFFB::new_effect(FFB_CreateNewEffect_Feature_Data_t* effect){
	// Only effect types advertised in the HID descriptor can be allocated.
	blockLoad_report.effectBlockIndex = 0;
	updatePoolAvailable();
	if(effect->effectType < FFB_EFFECT_CONSTANT || effect->effectType > FFB_EFFECT_FRICTION){
		blockLoad_report.loadStatus = 3;
		return;
	}

	int32_t index = effects_calc->find_free_effect(effect->effectType); // next effect
	if(index == -1){
		blockLoad_report.loadStatus = 2;
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Can't allocate a new effect");
#endif
		return;
	}
	FFB_Effect new_effect;
	new_effect.type = effect->effectType;

	this->effects_calc->logEffectType(effect->effectType,false);
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("New effect type:" + std::to_string(effect->effectType) + " idx: " + std::to_string(index-1));
#endif

	set_filters(&new_effect);

	effects[index] = std::move(new_effect);
	// Set block load report
	//reportFFBStatus.effectBlockIndex = index;
	blockLoad_report.effectBlockIndex = index+1;
	updatePoolAvailable();
	blockLoad_report.loadStatus = 1;
	sendStatusReport(index+1);
	

}

/**
 * Sets up an effect
 * If the direction enable bit is set then only 1 condition block is used
 */
void HidFFB::set_effect(FFB_SetEffect_t* effect){
	uint8_t index = effect->effectBlockIndex;
	if(index > effects.size() || index == 0 || effects[index-1].type == FFB_EFFECT_NONE
		|| effect->effectType < FFB_EFFECT_CONSTANT || effect->effectType > FFB_EFFECT_FRICTION)
		return;

	FFB_Effect* effect_p = &effects[index-1];

	if (effect_p->type != effect->effectType){
		effect_p->startTime = 0;
		set_filters(effect_p);
	}

	effect_p->gain = effect->gain;
	effect_p->type = effect->effectType;
	effect_p->samplePeriod = effect->samplePeriod;

	bool directionEnable = (effect->enableAxis & this->directionEnableMask);
	bool overridesCondition = false;

//	if(effect_p->useSingleCondition){ // Only allow turning single condition off in case it was overridden by sending multiple conditions previously
//		effect_p->useSingleCondition = directionEnable; // If direction is used only a single parameter block is allowed. Somehow this is still set while 2 conditions are sent...
//	}
	// Conditional effects usually do not use directions
	if(!effect_p->useSingleCondition && (effect->effectType == FFB_EFFECT_SPRING || effect->effectType == FFB_EFFECT_DAMPER || effect->effectType == FFB_EFFECT_INERTIA || effect->effectType == FFB_EFFECT_FRICTION))
	{
		if(effect_p->conditions[0].isActive()){
			effect_p->axisMagnitudes[0] = 1.0f;
			overridesCondition = true;
		}
		if(effect_p->conditions[1].isActive() || effect_p->useSingleCondition){
			effect_p->axisMagnitudes[1] = 1.0f;
			overridesCondition = true;
		}
	}


	if(!overridesCondition){
		float phaseX = M_PI*2.0 * (effect->directionX/36000.0f);

		effect_p->axisMagnitudes[0] = directionEnable ? sin(phaseX) : (effect->enableAxis & X_AXIS_ENABLE ? (effect->directionX - 18000.0f) / 18000.0f : 0); // Angular vector if dirEnable used otherwise linear or 0 if axis enabled
		effect_p->axisMagnitudes[1] = directionEnable ? -cos(phaseX) : (effect->enableAxis & Y_AXIS_ENABLE ? -(effect->directionY - 18000.0f) / 18000.0f : 0);
	}

#if MAX_AXIS == 3
	float phaseY = M_PI*2.0 * (effect->directionY/36000.0);
	effect_p->axisMagnitudes[3] = directionEnable ? sin(phaseY) : (effect->enableAxis & Z_AXIS_ENABLE ? (effect->directionZ - 18000.0f) / 18000.0f : 0);
#endif
	if(effect->duration == 0){ // Fix for games assuming 0 is infinite
		effect_p->duration = FFB_EFFECT_DURATION_INFINITE;
	}else{
		effect_p->duration = effect->duration;
	}
	effect_p->startDelay = effect->startDelay;
	if(!ffb_active)
		start_FFB();

	sendStatusReport(effect->effectBlockIndex);
	//CommandHandler::logSerialDebug("Setting Effect: " + std::to_string(effect->effectType) +  " at " + std::to_string(index) + "\n");
}

/**
	If the number of Condition report blocks is equal to the number of axes for the effect, then the first report
	block applies to the first axis, the second applies to the second axis, and so on. For example, a two-axis
	spring condition with CP Offset set to zero in both Condition report blocks would have the same effect as
	the joystick self-centering spring. When a condition is defined for each axis in this way, the effect must
	not be rotated.

	If there is a single Condition report block for an effect with more than one axis, then the direction along
	which the parameters of the Condition report block are in effect is determined by the direction parameters
	passed in the Direction field of the Effect report block. For example, a friction condition rotated 45
	degrees (in polar coordinates) would resist joystick motion in the northeast-southwest direction but would
	have no effect on joystick motion in the northwest-southeast direction.
 */
void HidFFB::set_condition(FFB_SetCondition_Data_t *cond){
	if(cond->effectBlockIndex == 0 || cond->effectBlockIndex > effects.size()){
		return;
	}
	// Exclude the type-specific ordinal fields: top two bits (1 axis), four (2 axes).
	const uint8_t axis = cond->parameterBlockOffset & (axisCount == 1 ? 0x3f : 0x0f);
	if(axis >= axisCount || axis >= MAX_AXIS) return;

	FFB_Effect *effect = &effects[cond->effectBlockIndex - 1];
	effect->conditions[axis].cpOffset = cond->cpOffset;
	effect->conditions[axis].negativeCoefficient = cond->negativeCoefficient;
	effect->conditions[axis].positiveCoefficient = cond->positiveCoefficient;
	effect->conditions[axis].negativeSaturation = cond->negativeSaturation;
	effect->conditions[axis].positiveSaturation = cond->positiveSaturation;
	effect->conditions[axis].deadBand = cond->deadBand;

//	if(effect->conditions[axis].positiveSaturation == 0){
//		effect->conditions[axis].positiveSaturation = 0x7FFF;
//	}
//	if(effect->conditions[axis].negativeSaturation == 0){
//		effect->conditions[axis].negativeSaturation = 0x7FFF;
//	}

	if(axis>0 && axis < MAX_AXIS && effect->conditions[axis].isActive()){ // Workaround when direction enable is set but multiple conditions are defined... Resets direction and uses conditions again
		effect->useSingleCondition = false;
	}
	if((effect->conditions[axis].isActive() || (axis > 0 && effect->useSingleCondition))  && effect->axisMagnitudes[axis] == 0){
		effect->axisMagnitudes[axis] = 1.0;
	}

//	for(uint8_t i = 0;i<MAX_AXIS;i++){
//		effect->axisMagnitudes[i] = 1.0;
//	}
}

void HidFFB::set_effect_operation(FFB_EffOp_Data_t* report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > effects.size()
		|| effects[report->effectBlockIndex-1].type == FFB_EFFECT_NONE
		|| report->state < 1 || report->state > 3){
		return; // Invalid ID or operation
	}
	// Start or stop effect
	uint8_t id = report->effectBlockIndex-1;
	if(report->state == 3){
		effects[id].state = 0; //Stop
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Stop effect: " + std::to_string(id));
#endif

	}else{

		// 1 = start, 2 = start solo
		if(report->state == 2){
#ifdef DEBUGLOG
			CommandHandler::logSerialDebug("Start solo: " + std::to_string(id));
#endif
			for(FFB_Effect& effect : effects){
				effect.state = 0; // Stop all other effects
			}
		}
		if(effects[id].state != 1){
			set_filters(&effects[id]);
		}
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Start effect: " + std::to_string(id));
#endif
		effects[id].startTime = (paused ? pausedAt : HAL_GetTick()) + effects[id].startDelay; // + effects[id].startDelay;
		effects[id].state = 1; //Start


	}
	sendStatusReport(report->effectBlockIndex);
	this->effects_calc->logEffectState(effects[id].type,effects[id].state);
}


void HidFFB::set_envelope(FFB_SetEnvelope_Data_t *report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > effects.size()){
		return;
	}
	FFB_Effect *effect = &effects[report->effectBlockIndex - 1];

	effect->attackLevel = report->attackLevel;
	effect->attackTime = report->attackTime;
	effect->fadeLevel = report->fadeLevel;
	effect->fadeTime = report->fadeTime;
	effect->useEnvelope = true;
}

void HidFFB::set_ramp(FFB_SetRamp_Data_t *report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > effects.size()){
		return;
	}
	FFB_Effect *effect = &effects[report->effectBlockIndex - 1];
	effect->magnitude = 0x7fff; // Full magnitude for envelope calculation. This effect does not have a periodic report
	effect->startLevel = report->startLevel;
	effect->endLevel = report->endLevel;
}

void HidFFB::set_periodic(FFB_SetPeriodic_Data_t* report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > effects.size()){
		return;
	}
	FFB_Effect* effect = &effects[report->effectBlockIndex-1];

	effect->period = clip<uint32_t,uint32_t>(report->period,1,0x7fff); // Period is never 0
	effect->magnitude = report->magnitude;
	effect->offset = report->offset;
	effect->phase = report->phase;
	//effect->counter = 0;
}


void HidFFB::updatePoolAvailable(){
	uint16_t available = 0;
	for(const auto& effect : effects){
		if(effect.type == FFB_EFFECT_NONE) ++available;
	}
	blockLoad_report.ramPoolAvailable = available * sizeof(FFB_Effect);
}

void HidFFB::free_effect(uint16_t id){
	if(id == 0 || id > effects.size() || effects[id-1].type == FFB_EFFECT_NONE) return;
	effects_calc->free_effect(id-1);
	updatePoolAvailable();
	sendStatusReport(0);
}

void HidFFB::reset_ffb(){
	for(uint8_t i=0;i<effects.size();i++) effects_calc->free_effect(i);
	hostDisabled = false;
	paused = false;
	blockLoad_report.effectBlockIndex = 0;
	blockLoad_report.loadStatus = 1;
	updatePoolAvailable();
	updateStatusReport();
}
