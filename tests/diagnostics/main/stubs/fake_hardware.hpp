
#pragma once
#include <cstdint>
#include "esp_err.h"
namespace fake {
inline int operation=0, fail_operation=0, raw_reads=0, fail_channel=-1, convert_channel=-1;
inline int mv[4]={1650,1650,1650,1650}, noise=0;
inline std::int64_t now=1000, read_delay=0;
inline int gpio_error=0, enable_calls=0, driver_failure=-1, align_calls=0, align_fail=-1, i2c_error=0;
inline int step() { return ++operation == fail_operation ? ESP_ERR_NO_MEM : ESP_OK; }
inline void reset() { operation=fail_operation=raw_reads=0; fail_channel=convert_channel=-1; noise=0; now=1000; read_delay=0; gpio_error=enable_calls=0; driver_failure=align_fail=-1; align_calls=i2c_error=0; for(auto &v:mv)v=1650; }
}
using gpio_num_t=int;
inline constexpr int GPIO_NUM_0=0,GPIO_NUM_2=2,GPIO_NUM_5=5,GPIO_NUM_12=12,GPIO_NUM_13=13,GPIO_NUM_14=14,GPIO_NUM_15=15,GPIO_NUM_18=18,GPIO_NUM_19=19,GPIO_NUM_23=23,GPIO_NUM_25=25,GPIO_NUM_26=26,GPIO_NUM_27=27,GPIO_NUM_32=32,GPIO_NUM_33=33,GPIO_NUM_34=34,GPIO_NUM_35=35,GPIO_NUM_36=36,GPIO_NUM_39=39;
inline constexpr int GPIO_MODE_OUTPUT=1,GPIO_PULLUP_ENABLE=1;
inline int gpio_set_level(int,int level) { if(level)++fake::enable_calls; return fake::gpio_error; }
inline int gpio_set_direction(int,int) { return fake::gpio_error; }
using adc_oneshot_unit_handle_t=void*; using adc_cali_handle_t=void*; using adc_channel_t=int; using adc_unit_t=int;
inline constexpr int ADC_UNIT_1=1,ADC_ATTEN_DB_12=12,ADC_BITWIDTH_12=12;
struct adc_oneshot_unit_init_cfg_t {int unit_id;};
struct adc_oneshot_chan_cfg_t {int atten,bitwidth;};
struct adc_cali_line_fitting_config_t {int unit_id,atten,bitwidth; unsigned default_vref;};
inline int adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t*,void **p) {*p=(void*)1;return fake::step();}
inline int adc_oneshot_io_to_channel(int pin,int *unit,int *ch) {*unit=1;*ch=pin==39?0:pin==36?1:pin==35?2:3;return fake::step();}
inline int adc_oneshot_config_channel(void*,int,const adc_oneshot_chan_cfg_t*) {return fake::step();}
inline int adc_cali_create_scheme_line_fitting(const adc_cali_line_fitting_config_t*,void **p) {*p=(void*)1;return fake::step();}
inline int adc_oneshot_read(void*,int ch,int *raw) {fake::now+=fake::read_delay;*raw=ch;++fake::raw_reads;return ch==fake::fail_channel?ESP_ERR_TIMEOUT:ESP_OK;}
inline int adc_cali_raw_to_voltage(void*,int ch,int *mv) {*mv=fake::mv[ch]+((fake::raw_reads/4)%2?fake::noise:0);return ch==fake::convert_channel?ESP_FAIL:ESP_OK;}
inline int adc_cali_delete_scheme_line_fitting(void*){return 0;}
inline int adc_oneshot_del_unit(void*){return 0;}
inline std::int64_t esp_timer_get_time(){return fake::now;}
inline void vTaskDelay(unsigned){fake::now+=1000;}
using i2c_port_t=int;using i2c_bus_handle_t=void*;using i2c_bus_device_handle_t=void*;
inline constexpr int I2C_NUM_0=0,I2C_NUM_1=1,I2C_MODE_MASTER=1;
struct i2c_config_t {int mode,sda_io_num,scl_io_num,sda_pullup_en,scl_pullup_en;struct {unsigned clk_speed;}master;};
inline void* i2c_bus_create(int,const i2c_config_t*){return (void*)1;}
inline void* i2c_bus_device_create(void*,int,int){return (void*)1;}
inline int i2c_bus_read_bytes(void*,int,int,std::uint8_t *raw){raw[0]=raw[1]=0;return fake::i2c_error;}
class Sensor {public:virtual float getSensorAngle()=0; void init(){getSensorAngle();} void update(){} float getMechanicalAngle(){return getSensorAngle();}};
class BLDCDriver3PWM {public: BLDCDriver3PWM(int,int,int){} float voltage_power_supply{},voltage_limit{}; int init(int i){return i!=fake::driver_failure;} void disable(){} void setPwm(float,float,float){} };
enum class MotionControlType {torque}; enum class TorqueControlType {voltage}; enum class FOCModulationType {SpaceVectorPWM}; enum class Direction {CW=1,CCW=-1};
class BLDCMotor {public:BLDCMotor(int){} MotionControlType controller{};TorqueControlType torque_controller{}; FOCModulationType foc_modulation{};float voltage_limit{},voltage_sensor_align{},zero_electric_angle{}; Direction sensor_direction{Direction::CW};void linkSensor(Sensor*){}void linkDriver(BLDCDriver3PWM*){} bool init(){return true;} bool initFOC(){return ++fake::align_calls!=fake::align_fail;} void disable(){} };
