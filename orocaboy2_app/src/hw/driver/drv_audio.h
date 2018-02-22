/*
 * drv_audio.h
 *
 *  Created on: Feb 21, 2018
 *      Author: opus
 */

#ifndef DRV_AUDIO_H_
#define DRV_AUDIO_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"


err_code_t drvAudioOutInit(uint32_t audio_freq);
void       drvAudioOutDeInit(void);

err_code_t drvAudioOutPlay(uint16_t* p_buf, uint32_t size);
err_code_t drvAudioOutPause(void);
err_code_t drvAudioOutResume(void);
err_code_t drvAudioOutStop(void);

err_code_t drvAudioOutSetVolume(uint8_t volume);
err_code_t drvAudioOutSetMute(uint32_t cmd);

void       drvAudioOutSetFrequency(uint32_t audio_freq);
void       drvAudioOutSetAudioFrameSlot(uint32_t frame_slot);

void       drvAudioOutChangeBuffer(uint16_t *p_data, uint16_t size);
void       drvAudioOutChangeConfig(uint32_t audio_out_option);



#ifdef __cplusplus
 }
#endif




#endif /* DRV_AUDIO_H_ */
