#ifndef _CONFIG_SOUND_GAMEBUINO_META_
#define _CONFIG_SOUND_GAMEBUINO_META_


/* SOUND Define */
#ifndef SOUND_CHANNELS
#define SOUND_CHANNELS 4
#endif

#ifndef SOUND_FREQ
#define SOUND_FREQ 44100
#endif

#ifndef SOUND_BUFFERSIZE
#define SOUND_BUFFERSIZE (2048 * SOUND_FREQ / 44100)
#endif



#endif /* _CONFIG_SOUND_GAMEBUINO_META_ */
