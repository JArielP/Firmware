#include <drivers/device/i2c.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <math.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <uORB/topics/angle_of_attack.h>

// 50Hz
#define CYCLE_TICKS_DELAY MSEC2TICK(20)


extern "C" { __EXPORT int aoas_main(int argc, char *argv[]); }

class AOAS {
public:
    AOAS(void);
    virtual ~AOAS();

    virtual int init();
    virtual int ioctl(file *filp, int cmd, unsigned long arg);
    virtual ssize_t read(file *filp, char *buffer, size_t len);

private:

    int fd;
    struct adc_msg_s adc_data[12];

    orb_advert_t	_topic;
    int         	_orb_class_instance;
    struct angle_of_attack_s orb_data;

    void cycle();
    static void cycle_trampoline(void *arg);

    struct work_s _work;

    math::LowPassFilter2p _filter_aoa;
};

static AOAS *instance = nullptr;


int aoas_main(int argc, char *argv[]) {
    const char *verb = argv[optind];

    if (!strcmp(verb, "start")) {
        if (instance) {
            warn("AOAS was already started");
            return PX4_OK;
        }

        instance = new AOAS();
        instance->init();

        if (!instance) {
            warn("AOAS not found");
            return PX4_ERROR;
        }

    } else {
        warn("Action not supported");
        return PX4_ERROR;
    }

    return PX4_OK;
}

AOAS::AOAS(void) :
        _topic(nullptr),
        _orb_class_instance(-1),
        _filter_aoa(50.0f, 1.0f) // sample_freq, cutoff_freq
{

}

AOAS::~AOAS() {
    px4_close(fd);
    if (_topic != nullptr) {
        orb_unadvertise(_topic);	//un-advertise uorb topic
    }
    work_cancel(HPWORK, &_work);	//stop the worker that calls the cycle function
}

int AOAS::init() {

    _topic = orb_advertise_multi(ORB_ID(angle_of_attack), &orb_data, &_orb_class_instance, ORB_PRIO_HIGH);		//advertise sensor uorb
    if (_topic == nullptr) { PX4_WARN("ADVERT FAIL");	}

    work_queue(HPWORK, &_work, (worker_t)&AOAS::cycle_trampoline, this, CYCLE_TICKS_DELAY);   //start worker to call cycle function
    return PX4_OK;
}

int AOAS::ioctl(file *filp, int cmd, unsigned long arg) {	return -ENOTTY; }	//px4 function that has to be here but does nothing (?)

ssize_t AOAS::read(file *filp, char *buffer, size_t len) { return len; }		//px4 function that has to be here but does nothing (?)

void AOAS::cycle_trampoline(void *arg) {						//trampoilne function running the cycle function
    AOAS *dev = reinterpret_cast<AOAS * >(arg);
    dev->cycle();
}

void AOAS::cycle() {

    fd = px4_open(ADC0_DEVICE_PATH, O_RDONLY);		//open adc device
    if (fd < 0) { warn("Error reading sample");  }
    ssize_t count = px4_read(fd, adc_data, sizeof(adc_data));	//read adc values
    if (count < 8) { warn("Error reading sample");  }

    //convert raw data to angle and save to uorb
    int raw = adc_data[7].am_data;
    orb_data.AoA_raw = raw;
    double raw_scale = ((double)raw - 2322.1)/439.52;
    orb_data.AoA = -1.5672*raw_scale*raw_scale*raw_scale - 2.9806*raw_scale*raw_scale - 31.253*raw_scale - 2.341 ;
    orb_data.AoA_filt = _filter_aoa.apply(orb_data.AoA);
    orb_data.timestamp = hrt_absolute_time();

    //PX4_INFO("%f", (double) orb_data.aoa);

    px4_close(fd);

    orb_publish(ORB_ID(angle_of_attack), _topic, &orb_data);	//publish data

    work_queue(HPWORK, &_work, (worker_t)&AOAS::cycle_trampoline, this, CYCLE_TICKS_DELAY);
}
