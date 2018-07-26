#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <math.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <uORB/topics/angle_of_attack.h>


#define SLAVE_ADDR 0x09

// cycle every 3 sec
#define CYCLE_TICKS_DELAY MSEC2TICK(100)

extern "C" { __EXPORT int aoas_2_main(int argc, char *argv[]); }

class AOAS_2 : public device::I2C {
public:
    AOAS_2();
    virtual ~AOAS_2();

    virtual int init();
    virtual int ioctl(file *filp, int cmd, unsigned long arg);
    virtual ssize_t read(file *filp, char *buffer, size_t len);

private:

    void cycle();
    static void cycle_trampoline(void *arg);

    orb_advert_t	_topic;
    int         	_orb_class_instance;
    struct angle_of_attack_s data;

    struct work_s _work;

    math::LowPassFilter2p _filter_aoa;
};

static AOAS_2 *instance = nullptr;


int aoas_2_main(int argc, char *argv[]) {
    const char *verb = argv[optind];

    if (!strcmp(verb, "start")) {
        if (instance) {	warn("AOAS_2 was already started"); return PX4_OK; }   //return if aoas_2 was already started

        instance = new AOAS_2();  //create new instance
        if (!instance) { warn("No memory to instance AOAS_2");	return PX4_ERROR; } //return if instance creation failed


        int ret = instance->init();   //initialize aoas_2
        if (ret != PX4_OK) {          //return and clean up if initialization fails
            warn("AOAS_2 not found");  //
            delete instance;            //
            instance = nullptr;         //
            return PX4_ERROR;           //
        }

        PX4_INFO("AOAS_2 succesfully started.");
        return PX4_OK;

    } else {	//unknown command
        warn("Action not supported");
        return PX4_ERROR;
    }
}

AOAS_2::AOAS_2() :
        I2C("AOAS_2", "/dev/aoas_2_ext", PX4_I2C_BUS_EXPANSION, SLAVE_ADDR, 100000),  //initialize i2c
        _topic(nullptr),
        _orb_class_instance(-1),
        _filter_aoa(50.0f, 5.0f) // sample_freq, cutoff_freq
{
//    data.is_new1 = false;		//no new data from sensor at startup
//    data.is_new2 = false;		//
}

AOAS_2::~AOAS_2() {
    if (_topic != nullptr) {
        orb_unadvertise(_topic);	//un-advertise uorb topic
    }
    work_cancel(HPWORK, &_work);	//stop the worker that calls the cycle function
}

int AOAS_2::init() {

    int ret = I2C::init();                //init i2c
    if (ret != PX4_OK) { return ret; }    //return if failed

    work_queue(HPWORK, &_work, (worker_t)&AOAS_2::cycle_trampoline, this, CYCLE_TICKS_DELAY);   //start worker to call cycle function

    _topic = orb_advertise_multi(ORB_ID(angle_of_attack), &data, &_orb_class_instance, ORB_PRIO_HIGH);		//advertise sensor uorb
    if (_topic == nullptr) { PX4_WARN("ADVERT FAIL");	}

    return PX4_OK;
}

int AOAS_2::ioctl(file *filp, int cmd, unsigned long arg) { return -ENOTTY; }	//px4 function that has to be here but does nothing (?)

ssize_t AOAS_2::read(file *filp, char *buffer, size_t len) { return len; }			//px4 function that has to be here but does nothing (?)

void AOAS_2::cycle_trampoline(void *arg) {						//trampoilne function running the cycle function
    AOAS_2 *dev = reinterpret_cast<AOAS_2 * >(arg);
    dev->cycle();
}

void AOAS_2::cycle() {
    uint8_t buffer[4];		//recv buffer for sensor data
    int ret;							//

    buffer[0] = 0x02;  //regirstry address // 1 für letzter Wert, 2 für mean seit letztem poll
    ret = transfer(buffer, 1, buffer, 4);		//request data from sensor
    if (ret != PX4_OK) { warn("Error reading sample"); }

    int raw = *((uint32_t*)(buffer));
    data.AoA_raw = raw;
    double r_s = ((double)raw-2262.4)/1203.9;
    data.AoA = 0.0791  * r_s * r_s * r_s * r_s * r_s * r_s * r_s * r_s -
               0.0866  * r_s * r_s * r_s * r_s * r_s * r_s * r_s -
               0.3133  * r_s * r_s * r_s * r_s * r_s * r_s +
               0.8688  * r_s * r_s * r_s * r_s * r_s +
               0.8274  * r_s * r_s * r_s * r_s -
               0.0552  * r_s * r_s * r_s +
               2.3864  * r_s * r_s +
               29.6463 * r_s -
               2.7290;
    data.AoA_filt = _filter_aoa.apply(data.AoA);
    data.AoA_filt = raw;
    data.model = angle_of_attack_s::AOA_MODEL_2;
    data.timestamp = hrt_absolute_time();

    orb_publish(ORB_ID(angle_of_attack), _topic, &data);		//publish data

    work_queue(HPWORK, &_work, (worker_t)&AOAS_2::cycle_trampoline, this, CYCLE_TICKS_DELAY);	//set up worker for next cycle
}
