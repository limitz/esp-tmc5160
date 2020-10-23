#include <tmc5160.h>

void app_main()
{
	STEPPER.init();
	STEPPER.enable(true);

	for (;;)
	{
		STEPPER.update();
		
		vTaskDelay(100);
	}

	// for show
	STEPPER.deinit();
}
