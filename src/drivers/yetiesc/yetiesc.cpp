
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/sensor_yetiesc.h>


#define SLAVE_ADDR 0x08

// cycle every 3 sec
#define CYCLE_TICKS_DELAY MSEC2TICK(3000)

extern "C" { __EXPORT int yetiesc_main(int argc, char *argv[]); }

class YETIESC : public device::I2C {
public:
	YETIESC();
	virtual ~YETIESC();

	virtual int init();
	virtual int ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t read(file *filp, char *buffer, size_t len);

private:

	void cycle();
	static void cycle_trampoline(void *arg);

  orb_advert_t	_topic;
	int         	_orb_class_instance;
	struct sensor_yetiesc_s data;

	struct work_s _work;
};

static YETIESC *instance = nullptr;


int yetiesc_main(int argc, char *argv[]) {
	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (instance) {	warn("YETIESC was already started"); return PX4_OK; }   //return if yetiesc was already started

		instance = new YETIESC();  //create new instance
		if (!instance) { warn("No memory to instance YETIESC");	return PX4_ERROR; } //return if instance creation failed


    int ret = instance->init();   //initialize yetiesc
    if (ret != PX4_OK) {          //return and clean up if initialization fails
      warn("YETIESC not found");  //
  		delete instance;            //
  		instance = nullptr;         //
      return PX4_ERROR;           //
		}

	} else {	//unknown command
		warn("Action not supported");
		return PX4_ERROR;
	}
  PX4_INFO("YETIESC succesfully started.");
	return PX4_OK;
}

YETIESC::YETIESC() :
	I2C("YETIESC", "/dev/yetiesc_ext", PX4_I2C_BUS_EXPANSION, SLAVE_ADDR, 100000),  //initialize i2c
  _topic(nullptr),
	_orb_class_instance(-1)
{
	data.is_new1 = false;		//no new data from sensor at startup
	data.is_new2 = false;		//
}

YETIESC::~YETIESC() {
  if (_topic != nullptr) {
		orb_unadvertise(_topic);	//un-advertise uorb topic
	}
	work_cancel(HPWORK, &_work);	//stop the worker that calls the cycle function
}

int YETIESC::init() {

	int ret = I2C::init();                //init i2c
	if (ret != PX4_OK) { return ret; }    //return if failed

	work_queue(HPWORK, &_work, (worker_t)&YETIESC::cycle_trampoline, this, CYCLE_TICKS_DELAY);   //start worker to call cycle function

	_topic = orb_advertise_multi(ORB_ID(sensor_yetiesc), &data, &_orb_class_instance, ORB_PRIO_HIGH);		//advertise sensor uorb
	if (_topic == nullptr) { PX4_WARN("ADVERT FAIL");	}

	return PX4_OK;
}

int YETIESC::ioctl(file *filp, int cmd, unsigned long arg) { return -ENOTTY; }	//px4 function that has to be here but does nothing (?)

ssize_t YETIESC::read(file *filp, char *buffer, size_t len) { return len; }			//px4 function that has to be here but does nothing (?)

void YETIESC::cycle_trampoline(void *arg) {						//trampoilne function running the cycle function
	YETIESC *dev = reinterpret_cast<YETIESC * >(arg);
	dev->cycle();
}

void YETIESC::cycle() {
	uint8_t buffer[28];		//recv buffer for sensor data
	int ret;							//

	buffer[0] = 0x42;  //regirstry address
	ret = transfer(buffer, 1, buffer, 28);		//request data from sensor
	if (ret != PX4_OK) { warn("Error reading sample"); }

	//read data from esc1
	data.voltage1 = *((uint32_t*)(buffer));
	data.mode1 = *((char*)(buffer+4));
	data.percent1 = *((uint16_t*)(buffer+5));
	data.temp1 = *((uint16_t*)(buffer+7));
	data.current1 = *((uint16_t*)(buffer+9));
	data.rpm1 = *((uint16_t*)(buffer+11));
	data.is_new1 = *((char*)(buffer+13));

	//read data from esc2
	data.voltage2 = *((uint32_t*)(buffer+14));
	data.mode2 = *((char*)(buffer+14+4));
	data.percent2 = *((uint16_t*)(buffer+14+5));
	data.temp2 = *((uint16_t*)(buffer+14+7));
	data.current2 = *((uint16_t*)(buffer+14+9));
	data.rpm2 = *((uint16_t*)(buffer+14+11));
	data.is_new2 = *((char*)(buffer+14+13));

	data.timestamp = hrt_absolute_time();

  //PX4_INFO("ESC1: v:%d, m:%c, p:%d, t:%d, c:%d, r:%d, n:%d", data.voltage1, data.mode1, data.percent1, data.temp1, data.current1, data.rpm1, data.is_new1);
	//PX4_INFO("ESC2: v:%d, m:%c, p:%d, t:%d, c:%d, r:%d, n:%d", data.voltage2, data.mode2, data.percent2, data.temp2, data.current2, data.rpm2, data.is_new2);

	if (data.is_new1 || data.is_new2) {		//if sensor provided new data
		orb_publish(ORB_ID(sensor_yetiesc), _topic, &data);		//publish data
	}

	work_queue(HPWORK, &_work, (worker_t)&YETIESC::cycle_trampoline, this, CYCLE_TICKS_DELAY);	//set up worker for next cycle
}
