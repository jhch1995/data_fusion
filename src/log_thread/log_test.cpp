#include "log.h"
void *f(void* args)
{
	for(int i = 0;i < 100; i++)
	{
			Log::get_instance()->write_log(1, "d=%d,c=%c,s=%s,f=%f", i,'a',"log", 1.000);
			Log::get_instance()->write_log(2, "d=%d,c=%c,s=%s,f=%f", i,'a',"log", 1.000);
			Log::get_instance()->write_log(3, "d=%d,c=%c,s=%s,f=%f", i,'a',"log", 1.000);

			LOG_INFO("%d", 123456789);
			LOG_ERROR("%d", 123456789);
			LOG_DEBUG("%d", 123456789);
			LOG_WARN("%d", 123456789);

	}
}

int main()
{
	Log::get_instance()->init("./mylog.log", 100, 2000000, 10);
	//Log::get_instance()->init("./mylog.log", 100, 2000000, 0);//synchronization model
	sleep(1);
	int i = 0;
	Log::get_instance()->write_log(1, "d=%d,c=%c,s=%s,f=%f", i,'a',"log", 1.000);
	Log::get_instance()->write_log(2, "d=%d,c=%c,s=%s,f=%f", i,'a',"log", 1.000);
	Log::get_instance()->write_log(3, "d=%d,c=%c,s=%s,f=%f", i,'a',"log", 1.000);

	LOG_INFO("%d", 123456789);
	LOG_ERROR("%d", 123456789);
	LOG_DEBUG("%d", 123456789);
	LOG_WARN("%d", 123456789);

	pthread_t id;
	for(int i = 0; i < 1; i++)
	{
		pthread_create(&id, NULL, f, NULL);
		pthread_join(id,NULL);
	}

	//for(;;)
	{
		sleep(15);
		Log::get_instance()->flush();
	}

	return 0;
}
