#include <tmc5160.h>

#define UBIT(b, v) ((v) & ((1<<(b))-1))

void app_main()
{
	STEPPER.init();
	STEPPER.reset(0);
	STEPPER.enable(true);

	/*
	tmc5160_t* m = STEPPER.motors;


	m->thresholds.stealth = 0x00001000;
	m->thresholds.tcool   = 0x00001000;
	m->thresholds.thigh   = 0x00000100;
	m->thresholds.dcstep  = UBIT(23, 0x8000);
	m->thresholds.v1      = 0x00008000;

	m->vstart   = UBIT(18,   0x500);
	m->vstop    = UBIT(18,   0x600);
	m->a1       = UBIT(16,   0x600);
	m->d1       = UBIT(16,   0x800);
	m->v1       = UBIT(20, 0x10000);
	m->amax     = UBIT(16,   0x300);
	m->vmax     = UBIT(23, 0x30000);
	m->dmax     = UBIT(16,   0xF00);

	m->tzerowait = UBIT(16, 0x4000);
	//m->xtarget = UBIT(32, 0x0000);
	*/

	for (;;)
	{
		STEPPER.motors[0].target_position += 0x00400000;
		STEPPER.update();
		vTaskDelay(1500);
	}

	// for show
	STEPPER.deinit();
}
