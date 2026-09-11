#include "unity.h"
#include "diagnostic_store.hpp"
#include "ble_status_codec.hpp"
#include "balance_controller.hpp"
#include "remote_protocol_checks.cpp"
#include "../../../components/BSP/Motor/current_sense.cpp"
#include "../../../components/BSP/Motor/motor_foc_service.cpp"
#include "../../../components/BSP/IMU/bmi160_attitude.cpp"
using namespace vehicle;
namespace cs=vehicle::motor::current_sense;
void resetMotor()
{
    fake::reset();
    motor::initialized=false; motor::stopped=false; motor::enable_ready=false;
    motor::outputs_enabled=false; motor::left_driver_ready=false; motor::right_driver_ready=false;
    motor::wheel_sample_ready=false; motor::previous_current_us=0; motor::previous_encoder_us=0;
    motor::left_state={};motor::right_state={};
    motor::left_sensor.healthy=true;motor::right_sensor.healthy=true;
    motor::left_sensor.cached=false;motor::right_sensor.cached=false;
}
void errorsAndRing()
{
    ErrorInfo error{};
    VEHICLE_ERROR(&error,ESP_ERR_TIMEOUT,current_raw,esp,123,42,99,2,7,1);
    TEST_ASSERT_EQUAL(123,error.raw_code);TEST_ASSERT_NOT_NULL(error.file);TEST_ASSERT_TRUE(error.line>0);
    diagnostics::CrashState s{};s.last_control.sequence=91;
    diagnostics::commit(s,error,true);
    auto first=s.first_fault;
    for(int n=0;n<40;++n) { diagnostics::commit(s,{},true); }
    TEST_ASSERT_EQUAL(41,s.event_seq);TEST_ASSERT_EQUAL(16,s.count);
    TEST_ASSERT_EQUAL(first.event_seq,s.first_fault.event_seq);
    TEST_ASSERT_EQUAL(91,s.fault_control.sequence);
    diagnostics::Event event{};TEST_ASSERT_TRUE(diagnostics::nextEvent(s,0,event));TEST_ASSERT_EQUAL(26,event.event_seq);
    TEST_ASSERT_FALSE(diagnostics::nextEvent(s,41,event));
}
void wirePackets()
{
    diagnostics::Event e{}; e.event_seq=0x12345678; e.error.point_id=ErrorPoint::current_raw;
    e.error.raw_code=-123; e.error.domain=ErrorDomain::esp;e.error.channel=3;e.flags=3;
    std::uint8_t p[14]{};diagnostics::encodeEvent(e,p);
    const std::uint8_t expected[14]={1,4,2,0x78,0x56,0x34,0x12,1,3,0x85,0xff,0xff,0xff,3};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected,p,14);
    diagnostics::CrashState s{};s.boot_complete=true;s.event_seq=0x12345678;s.count=16;s.mtu=247;
    std::uint8_t m[20]{};diagnostics::encodeMetadata(s,m);
    TEST_ASSERT_EQUAL(1,m[0]);TEST_ASSERT_EQUAL(1,m[1]);TEST_ASSERT_EQUAL(0x78,m[4]);TEST_ASSERT_EQUAL(247,m[16]);TEST_ASSERT_EQUAL(16,m[14]);
}
void gates()
{
    ble::RemoteState s{};ble::remoteConnection(s,true);ble::stepRemote(s,0);
    TEST_ASSERT_TRUE(ble::acceptCommand(s,{'D',1,0,0,false},1));ble::stepRemote(s,1);
    TEST_ASSERT_FALSE(ble::acceptCommand(s,{'A',2,0,0,false},2));
    s.boot_complete=s.run_allowed=true;ble::stepRemote(s,3);TEST_ASSERT_NOT_EQUAL(static_cast<int>(ble::RemoteMode::active),static_cast<int>(s.mode));
    TEST_ASSERT_TRUE(ble::acceptCommand(s,{'A',3,0,0,false},4));ble::stepRemote(s,4);TEST_ASSERT_EQUAL(static_cast<int>(ble::RemoteMode::active),static_cast<int>(s.mode));
    s.fault=true;ble::remoteConnection(s,false);ble::remoteConnection(s,true);ble::stepRemote(s,5);
    TEST_ASSERT_TRUE(s.fault);TEST_ASSERT_FALSE(ble::acceptCommand(s,{'A',4,0,0,false},6));
    ble::RemoteCommand command{};TEST_ASSERT_FALSE(ble::parseCommand("0,0",true,command));
}
void adcInitializationFailures()
{
    const ErrorPoint points[]={ErrorPoint::current_unit,ErrorPoint::current_map,ErrorPoint::current_channel,
        ErrorPoint::current_map,ErrorPoint::current_channel,ErrorPoint::current_map,ErrorPoint::current_channel,
        ErrorPoint::current_map,ErrorPoint::current_channel,ErrorPoint::current_calibration};
    for(int n=1;n<=10;++n) {
        fake::reset();fake::fail_operation=n;ErrorInfo e{};
        TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM,cs::initialize(&e));TEST_ASSERT_EQUAL(static_cast<int>(points[n-1]),static_cast<int>(e.point_id));TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM,e.raw_code);
    }
}
void adcRuntimeFailures()
{
    fake::reset();ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,cs::initialize(&e));
    for(int ch=0;ch<4;++ch) {
        cs::Sample sample{};fake::fail_channel=ch;
        TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,cs::read(&sample,&e));TEST_ASSERT_FALSE(sample.valid);TEST_ASSERT_EQUAL(ch,e.channel);TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::current_raw),static_cast<int>(e.point_id));
        fake::fail_channel=-1;fake::convert_channel=ch;
        TEST_ASSERT_EQUAL(ESP_FAIL,cs::read(&sample,&e));TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::current_mv),static_cast<int>(e.point_id));fake::convert_channel=-1;
        fake::mv[ch]=149;TEST_ASSERT_NOT_EQUAL(ESP_OK,cs::read(&sample,&e));TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::current_range),static_cast<int>(e.point_id));
        fake::mv[ch]=2350;TEST_ASSERT_NOT_EQUAL(ESP_OK,cs::read(&sample,&e));TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::phase_limit),static_cast<int>(e.point_id));fake::mv[ch]=1650;
    }
    for(int motor=0;motor<2;++motor) {
        fake::mv[2*motor]=fake::mv[2*motor+1]=2050;cs::Sample sample{};
        TEST_ASSERT_NOT_EQUAL(ESP_OK,cs::read(&sample,&e));TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::reconstructed_limit),static_cast<int>(e.point_id));TEST_ASSERT_EQUAL(motor,e.channel);
        fake::mv[2*motor]=fake::mv[2*motor+1]=1650;
    }
    fake::read_delay=501;cs::Sample sample{};TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,cs::read(&sample,&e));TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::current_timeout),static_cast<int>(e.point_id));
}
void offsets()
{
    ErrorInfo e{};fake::reset();fake::mv[2]=1200;TEST_ASSERT_NOT_EQUAL(ESP_OK,cs::initialize(&e));TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::offset_mean),static_cast<int>(e.point_id));
    fake::reset();fake::noise=101;TEST_ASSERT_NOT_EQUAL(ESP_OK,cs::initialize(&e));TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::offset_noise),static_cast<int>(e.point_id));
}
void motorFailures()
{
    for(int side=0;side<2;++side) {
        resetMotor();fake::driver_failure=side;ErrorInfo e{};TEST_ASSERT_NOT_EQUAL(ESP_OK,motor::initialize(&e));
        TEST_ASSERT_EQUAL(static_cast<int>(side?ErrorPoint::right_driver:ErrorPoint::left_driver),static_cast<int>(e.point_id));TEST_ASSERT_EQUAL(0,fake::enable_calls);
    }
    for(int align=1;align<=2;++align) {
        resetMotor();fake::align_fail=align;ErrorInfo e{};TEST_ASSERT_NOT_EQUAL(ESP_OK,motor::initialize(&e));
        TEST_ASSERT_EQUAL(static_cast<int>(align==1?ErrorPoint::right_alignment:ErrorPoint::left_alignment),static_cast<int>(e.point_id));
    }
    resetMotor();fake::gpio_error=ESP_FAIL;ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_FAIL,motor::inhibitOutputs(&e));TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::disable_gpio),static_cast<int>(e.point_id));
}
void failedSamplesDoNotCommit()
{
    resetMotor();ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,motor::initialize(&e));
    motor::WheelState wheel{};TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheel,&e));
    motor::CurrentFeedback current{};TEST_ASSERT_EQUAL(ESP_OK,motor::runCurrentControl({0,0},&current,&e));
    const auto previous=motor::previous_current_us;
    fake::now+=1000;TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheel,&e));fake::fail_channel=2;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,motor::runCurrentControl({0,0},&current,&e));TEST_ASSERT_EQUAL(previous,motor::previous_current_us);TEST_ASSERT_FALSE(current.valid);
    const auto encoder_time=motor::previous_encoder_us;
    fake::now+=1000;fake::i2c_error=ESP_ERR_TIMEOUT;TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,motor::readWheelState(&wheel,&e));TEST_ASSERT_EQUAL(encoder_time,motor::previous_encoder_us);
    fake::now+=1000;fake::i2c_error=ESP_OK;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,motor::readWheelState(&wheel,&e));
}
void replayRetriesAndReconnect()
{
    diagnostics::CrashState s{};
    diagnostics::commit(s,{},false);diagnostics::commit(s,{},true);
    for(int i=0;i<20;++i) { diagnostics::commit(s,{},false); }
    diagnostics::ReplayCursor c{};diagnostics::Event e{};bool first{};
    TEST_ASSERT_TRUE(diagnostics::prepareReplay(s,c,e,first));TEST_ASSERT_TRUE(first);TEST_ASSERT_EQUAL(2,e.event_seq);
    diagnostics::acceptReplay(c,e,first,false);
    TEST_ASSERT_TRUE(diagnostics::prepareReplay(s,c,e,first));TEST_ASSERT_EQUAL(2,e.event_seq);
    diagnostics::acceptReplay(c,e,first,true);
    TEST_ASSERT_TRUE(diagnostics::prepareReplay(s,c,e,first));TEST_ASSERT_FALSE(first);TEST_ASSERT_EQUAL(7,e.event_seq);
    diagnostics::acceptReplay(c,e,first,false);TEST_ASSERT_EQUAL(0,c.after);
    diagnostics::acceptReplay(c,e,first,true);TEST_ASSERT_EQUAL(7,c.after);
    c={};TEST_ASSERT_TRUE(diagnostics::prepareReplay(s,c,e,first));TEST_ASSERT_EQUAL(2,e.event_seq);TEST_ASSERT_EQUAL(16,s.count);
}
void statusCompatibility()
{
    ble::StatusSnapshot status{};ble::RemoteState remote{};
    remote.connected=true;remote.has_command=true;remote.received_us=1000;
    status.command.connection_epoch=7;status.command.sequence=0x1234;status.command.sequence_valid=true;
    status.command.mode=ble::RemoteMode::active;status.sampled_us=1000;status.sensors_valid=true;
    status.pitch_deg=1.25f;status.left_velocity_rad_s=-2.5f;status.right_velocity_rad_s=3.0f;status.sample_sequence=0xabcd;
    std::uint8_t p[20]{};ble::encodeStatusPacket(p,status,remote,7,2000);
    TEST_ASSERT_EQUAL(2,p[0]);TEST_ASSERT_EQUAL(0x34,p[4]);TEST_ASSERT_EQUAL(0x12,p[5]);
    TEST_ASSERT_EQUAL(125,p[8]);TEST_ASSERT_EQUAL(0x06,p[10]);TEST_ASSERT_EQUAL(0xff,p[11]);TEST_ASSERT_EQUAL(0xcd,p[18]);
    status.pitch_deg=std::bit_cast<float>(0x7fc00000U);ble::encodeStatusPacket(p,status,remote,7,2000);
    TEST_ASSERT_EQUAL(0,p[2]&4);TEST_ASSERT_EQUAL(0,p[8]);
    ble::encodeStatusPacket(p,status,remote,8,2000);TEST_ASSERT_EQUAL(0,p[4]);TEST_ASSERT_EQUAL(0,p[2]&8);
}
void outputAgeStages()
{
    for (const bool before : {true,false}) {
        resetMotor(); ErrorInfo e{}; TEST_ASSERT_EQUAL(ESP_OK,motor::initialize(&e));
        const auto enables=fake::enable_calls;
        motor::WheelState wheel{};TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheel,&e));
        // 模拟编码器到输出之间的等待，ADC本身仍在独立的2ms预算内。
        fake::now+=before ? 3800 : 3500;
        fake::read_delay=before ? 100 : 0;
        fake::pwm_delay=before ? 0 : 300;
        CurrentTiming timing{};motor::CurrentFeedback feedback{};
        TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,motor::runCurrentControl({0,0},&feedback,&e,&timing));
        TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::output_age),static_cast<int>(e.point_id));
        TEST_ASSERT_EQUAL(static_cast<int>(before ? CurrentStage::before_pwm : CurrentStage::after_pwm),static_cast<int>(timing.stage));
        TEST_ASSERT_TRUE((timing.encoder_age_us)==(before ? 4200 : 4100));
        TEST_ASSERT_TRUE((timing.current_age_us)==(before ? 400 : 600));
        TEST_ASSERT_TRUE((timing.adc_us)==(before ? 400 : 0));
        TEST_ASSERT_TRUE((timing.pwm_us)==(before ? 0 : 600));
        TEST_ASSERT_EQUAL_FLOAT(before ? 4200.0f : 4100.0f,e.value);
        TEST_ASSERT_EQUAL_FLOAT(4000.0f,e.threshold);TEST_ASSERT_EQUAL(3,e.valid_fields);
        TEST_ASSERT_EQUAL(before ? 0 : 2,fake::pwm_calls);TEST_ASSERT_EQUAL(enables,fake::enable_calls);
        TEST_ASSERT_FALSE(feedback.valid);TEST_ASSERT_TRUE((motor::previous_current_us)==(0));
    }
}
void outputAgeBoundary()
{
    for(const int age : {3999,4000,4001}) {
        resetMotor();ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,motor::initialize(&e));
        const auto enables=fake::enable_calls;
        motor::WheelState wheel{};TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheel,&e));
        fake::now+=age-1000;fake::read_delay=100;fake::pwm_delay=300;
        CurrentTiming timing{};motor::CurrentFeedback feedback{};
        TEST_ASSERT_EQUAL(age<=4000 ? ESP_OK : ESP_ERR_TIMEOUT,motor::runCurrentControl({0,0},&feedback,&e,&timing));
        TEST_ASSERT_TRUE(age==timing.encoder_age_us);
        TEST_ASSERT_EQUAL(age<=4000,feedback.valid);
        TEST_ASSERT_EQUAL(enables+(age<=4000 ? 1 : 0),fake::enable_calls);
        if(age>4000)TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::output_age),static_cast<int>(e.point_id));
    }
}
void currentOutputAgeStages()
{
    for(const bool before : {true,false}) {
        resetMotor();ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,motor::initialize(&e));
        const auto enables=fake::enable_calls;
        motor::WheelState wheel{};TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheel,&e));
        fake::now+=100;fake::read_delay=100;
        CurrentTiming timing{};fake::observed_current_timing=&timing;
        fake::math_delay=before ? 1601 : 0;fake::pwm_delay=before ? 0 : 801;
        motor::CurrentFeedback feedback{};
        TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,motor::runCurrentControl({0.2f,0.2f},&feedback,&e,&timing));
        TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::current_output_age),static_cast<int>(e.point_id));
        TEST_ASSERT_EQUAL(static_cast<int>(before ? CurrentStage::before_pwm : CurrentStage::after_pwm),static_cast<int>(timing.stage));
        TEST_ASSERT_TRUE((before ? 2001 : 2002)==timing.current_age_us);
        TEST_ASSERT_TRUE(timing.encoder_age_us<4000);
        TEST_ASSERT_EQUAL_FLOAT(2000,e.threshold);TEST_ASSERT_EQUAL(3,e.valid_fields);
        TEST_ASSERT_EQUAL(before ? 0 : 2,fake::pwm_calls);TEST_ASSERT_EQUAL(enables,fake::enable_calls);
        TEST_ASSERT_FALSE(feedback.valid);TEST_ASSERT_TRUE(0==motor::previous_current_us);
        TEST_ASSERT_EQUAL_FLOAT(0,motor::left_state.pi.integral_v);TEST_ASSERT_EQUAL_FLOAT(0,motor::right_state.pi.integral_v);
        fake::observed_current_timing=nullptr;
    }
}
void currentOutputAgeBoundary()
{
    for(const bool before : {true,false})for(const int age : {1999,2000,2001}) {
        resetMotor();ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,motor::initialize(&e));
        motor::WheelState wheel{};TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheel,&e));
        fake::read_delay=100;fake::pwm_delay=before ? 0 : 500;
        fake::math_delay=age-(before ? 400 : 1400);
        CurrentTiming timing{};fake::observed_current_timing=&timing;
        motor::CurrentFeedback feedback{};
        TEST_ASSERT_EQUAL(age<=2000 ? ESP_OK : ESP_ERR_TIMEOUT,motor::runCurrentControl({0,0},&feedback,&e,&timing));
        TEST_ASSERT_TRUE(age==timing.current_age_us);TEST_ASSERT_EQUAL(age<=2000,feedback.valid);
        if(age>2000)TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::current_output_age),static_cast<int>(e.point_id));
        fake::observed_current_timing=nullptr;
    }
}
void readBudgetsRemainTwoMilliseconds()
{
    for(const int delay : {1000,1001}) {
        resetMotor();ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,motor::initialize(&e));
        fake::encoder_read_delay=delay;motor::WheelState wheel{};
        TEST_ASSERT_EQUAL(delay==1000 ? ESP_OK : ESP_ERR_TIMEOUT,motor::readWheelState(&wheel,&e));
        TEST_ASSERT_EQUAL(delay==1000,wheel.valid);
        if(delay>1000){TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::wheel_age),static_cast<int>(e.point_id));TEST_ASSERT_EQUAL_FLOAT(2000,e.threshold);}
    }
    for(const int delay : {500,501}) {
        fake::reset();ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,cs::initialize(&e));
        fake::read_delay=delay;cs::Sample sample{};
        TEST_ASSERT_EQUAL(delay==500 ? ESP_OK : ESP_ERR_TIMEOUT,cs::read(&sample,&e));
        if(delay>500){TEST_ASSERT_EQUAL(static_cast<int>(ErrorPoint::current_timeout),static_cast<int>(e.point_id));TEST_ASSERT_EQUAL_FLOAT(2000,e.threshold);}
    }
}
void timingSnapshotsAndFirstRelease()
{
    TEST_ASSERT_TRUE((sampleInterval(13000000,0,1000))==(1000));
    TEST_ASSERT_TRUE((sampleInterval(13000000,13000000,1000))==(0));
    TEST_ASSERT_TRUE((sampleInterval(12999999,13000000,1000))==(-1));
    diagnostics::CrashState s{};ControlTiming timing{};
    timing.cycle=1;timing.stage=ControlStage::complete;diagnostics::commitTiming(s,timing);
    timing.cycle=22;timing.balancing=true;timing.starting=true;timing.balance_cycle=1;
    timing.stage=ControlStage::current;timing.current.stage=CurrentStage::before_pwm;
    timing.current.encoder_age_us=2200;diagnostics::commitTiming(s,timing);
    diagnostics::commit(s,{},true);
    TEST_ASSERT_EQUAL(1,s.first_loop.cycle);TEST_ASSERT_EQUAL(0,s.first_balance.cycle);
    TEST_ASSERT_EQUAL(22,s.fault_timing.cycle);TEST_ASSERT_TRUE((s.fault_timing.current.encoder_age_us)==(2200));
    timing.cycle=23;timing.stage=ControlStage::complete;diagnostics::commitTiming(s,timing);
    diagnostics::commit(s,{},true);
    TEST_ASSERT_EQUAL(22,s.fault_timing.cycle);TEST_ASSERT_EQUAL(23,s.first_balance.cycle);
}
void independentBalanceWithoutArm()
{
    ble::RemoteState remote{};ble::stepRemote(remote,1000);
    control::BalanceEnableState enable{};
    TEST_ASSERT_TRUE(control::stepBalanceEnable(enable,false,remote.stop_generation));
    control::ControllerState controller{},reference{};
    control::ControlInput input{0,0,config::kPitchOffsetRad+0.02f,0,1.0f,0.2f,true,false};
    auto centered=input;centered.throttle_velocity_rad_s=0;centered.yaw_rate_rad_s=0;
    control::ControlOutput output{};
    for(int n=0;n<20;++n) {
        output=control::update(controller,input,0.002f);
        const auto expected=control::update(reference,centered,0.002f);
        TEST_ASSERT_TRUE(output.valid);TEST_ASSERT_TRUE(output.balance_current_a>0);
        TEST_ASSERT_EQUAL_FLOAT(expected.left_target_a,output.left_target_a);
        TEST_ASSERT_EQUAL_FLOAT(expected.right_target_a,output.right_target_a);
    }
    TEST_ASSERT_EQUAL_FLOAT(0,controller.speed_reference_rad_s);
    TEST_ASSERT_EQUAL_FLOAT(0,controller.yaw_reference_rad_s);
    input.pitch_rad=config::kPitchOffsetRad;input.left_velocity_rad_s=input.right_velocity_rad_s=1;
    for(int n=0;n<6;++n)output=control::update(controller,input,0.002f);
    TEST_ASSERT_TRUE(output.target_pitch_rad<config::kPitchOffsetRad); // 未ARM时速度环仍在纠偏。
    input.balancing=false;output=control::update(controller,input,0.002f);
    TEST_ASSERT_TRUE(output.valid);TEST_ASSERT_EQUAL_FLOAT(0,output.left_target_a);
    TEST_ASSERT_EQUAL_FLOAT(0,output.right_target_a);TEST_ASSERT_EQUAL_FLOAT(0,controller.outer_elapsed_s);
}
void independentBalanceCurrentPath()
{
    resetMotor();ErrorInfo error{};TEST_ASSERT_EQUAL(ESP_OK,motor::initialize(&error));
    const auto enables=fake::enable_calls;
    motor::WheelState wheels{};TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheels,&error));
    control::ControllerState controller{};
    const auto output=control::update(controller,{0,0,config::kPitchOffsetRad+0.02f,0,0,0,true,false},0.002f);
    TEST_ASSERT_TRUE(output.valid);TEST_ASSERT_TRUE(output.left_target_a>0);
    CurrentTiming timing{};motor::CurrentFeedback feedback{};
    TEST_ASSERT_EQUAL(ESP_OK,motor::runCurrentControl({output.left_target_a,output.right_target_a},&feedback,&error,&timing));
    TEST_ASSERT_TRUE(feedback.valid);TEST_ASSERT_EQUAL(enables+1,fake::enable_calls);
    TEST_ASSERT_EQUAL(static_cast<int>(CurrentStage::complete),static_cast<int>(timing.stage));
}
void balanceStopRequiresNewArm()
{
    ble::RemoteState remote{};remote.boot_complete=remote.run_allowed=true;
    control::BalanceEnableState enable{};
    ble::remoteConnection(remote,true);ble::stepRemote(remote,0);
    TEST_ASSERT_TRUE(control::stepBalanceEnable(enable,false,remote.stop_generation));
    TEST_ASSERT_TRUE(ble::acceptCommand(remote,{'S',1,0,0,false},1));
    TEST_ASSERT_TRUE(ble::acceptCommand(remote,{'D',2,0,0,false},2));
    ble::stepRemote(remote,2);
    TEST_ASSERT_FALSE(control::stepBalanceEnable(enable,false,remote.stop_generation));
    ble::remoteConnection(remote,false);ble::remoteConnection(remote,true);ble::stepRemote(remote,3);
    TEST_ASSERT_FALSE(control::stepBalanceEnable(enable,false,remote.stop_generation));
    TEST_ASSERT_TRUE(ble::acceptCommand(remote,{'D',1,0,0,false},4));ble::stepRemote(remote,4);
    TEST_ASSERT_FALSE(control::stepBalanceEnable(enable,false,remote.stop_generation));
    TEST_ASSERT_TRUE(ble::acceptCommand(remote,{'A',2,0,0,false},5));ble::stepRemote(remote,5);
    TEST_ASSERT_EQUAL(static_cast<int>(ble::RemoteMode::active),static_cast<int>(remote.mode));
    TEST_ASSERT_TRUE(control::stepBalanceEnable(enable,true,remote.stop_generation));
    ble::stepRemote(remote,300005);
    TEST_ASSERT_FALSE(control::stepBalanceEnable(enable,false,remote.stop_generation));
    const auto stopped_at=remote.stop_generation;ble::stepRemote(remote,300006);
    TEST_ASSERT_EQUAL(stopped_at,remote.stop_generation);
}
void balanceStopSurvivesUnobservedReconnect()
{
    auto remote=ble::armed();control::BalanceEnableState enable{};
    TEST_ASSERT_TRUE(control::stepBalanceEnable(enable,true,remote.stop_generation));
    ble::remoteConnection(remote,false);ble::remoteConnection(remote,true);ble::stepRemote(remote,10);
    TEST_ASSERT_FALSE(control::stepBalanceEnable(enable,false,remote.stop_generation));
    remote=ble::armed();enable={};
    TEST_ASSERT_TRUE(ble::acceptCommand(remote,{'D',3,0,0,false},300002));ble::stepRemote(remote,300002);
    TEST_ASSERT_FALSE(control::stepBalanceEnable(enable,false,remote.stop_generation));
}
void firstAttitudeUsesMeasuredTilt()
{
    fake::reset();imu::initialized=true;imu::device=reinterpret_cast<void*>(1);imu::resetEstimator();
    for(auto &byte:fake::imu_raw)byte=0;
    const auto setWord=[](int offset,std::int16_t value){
        const auto bits=static_cast<std::uint16_t>(value);
        fake::imu_raw[offset]=bits;fake::imu_raw[offset+1]=bits>>8;
    };
    setWord(6,-11585);setWord(10,11585); // 静态45度，不能被首帧滤波压成0.9度。
    imu::AttitudeSample attitude{};ErrorInfo error{};
    fake::i2c_error=ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,imu::readAttitude(&attitude,&error));
    TEST_ASSERT_FALSE(attitude.valid);TEST_ASSERT_TRUE(imu::previous_sample_us==0);
    fake::i2c_error=ESP_OK;
    TEST_ASSERT_EQUAL(ESP_OK,imu::readAttitude(&attitude,&error));
    TEST_ASSERT_FLOAT_WITHIN(0.01f,45.0f,attitude.pitch_deg);
    TEST_ASSERT_TRUE(std::abs(attitude.pitch_deg*config::kDegToRad-config::kPitchOffsetRad)>config::kFallAngleRad);
    setWord(6,0);setWord(10,16384);fake::now+=2000;
    TEST_ASSERT_EQUAL(ESP_OK,imu::readAttitude(&attitude,&error));
    TEST_ASSERT_FLOAT_WITHIN(0.01f,44.1f,attitude.pitch_deg);
}
void attitudeFilterUsesActualInterval()
{
    for(const int interval : {2000,5000,10000}) {
        fake::reset();imu::initialized=true;imu::device=reinterpret_cast<void*>(1);imu::resetEstimator();
        // 实测首帧45度，随后加速度计回到0度，陀螺仪静止。
        const std::int16_t x=-11585,z=11585;
        fake::imu_raw[6]=x&255;fake::imu_raw[7]=static_cast<std::uint16_t>(x)>>8;
        fake::imu_raw[10]=z&255;fake::imu_raw[11]=z>>8;
        imu::AttitudeSample sample{};ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,imu::readAttitude(&sample,&e));
        for(auto &byte:fake::imu_raw){byte=0;}
        fake::imu_raw[11]=0x40;
        fake::now+=interval;TEST_ASSERT_EQUAL(ESP_OK,imu::readAttitude(&sample,&e));
        const float expected=interval==2000 ? 44.1f : interval==5000 ? 42.81553f : 40.83333f;
        TEST_ASSERT_FLOAT_WITHIN(0.001f,expected,sample.pitch_deg);
    }
}
void outerLoopAtTwoHundredHzAttitude()
{
    control::ControllerState state{};
    const float dt=config::kControlPeriodUs*config::kAttitudeDivider*1.0e-6f;
    TEST_ASSERT_FLOAT_WITHIN(0.000001f,0.005f,dt);
    const control::ControlInput input{1,1,config::kPitchOffsetRad,0,0,0,true,false};
    TEST_ASSERT_TRUE(control::update(state,input,dt).valid);
    TEST_ASSERT_EQUAL_FLOAT(0,state.target_pitch_rad);
    TEST_ASSERT_TRUE(control::update(state,input,dt).valid);
    TEST_ASSERT_TRUE(state.target_pitch_rad<0);TEST_ASSERT_EQUAL_FLOAT(0,state.outer_elapsed_s);
}
void imuConfiguresBothSensorsAtEightHundredHz()
{
    fake::reset();imu::initialized=false;
    // 寄存器fake只提供初始化所需的就绪应答，不模拟真实PMU/FOC时序。
    fake::imu_registers[0x00]=0xd1;fake::imu_registers[0x03]=0x14;fake::imu_registers[0x1b]=0x08;
    fake::imu_registers[0x40]=0x28;fake::imu_registers[0x42]=0x28;
    ErrorInfo e{};TEST_ASSERT_EQUAL(ESP_OK,imu::initialize(&e));
    TEST_ASSERT_EQUAL_HEX8(0x2b,fake::imu_registers[0x40]);
    TEST_ASSERT_EQUAL_HEX8(0x2b,fake::imu_registers[0x42]);
    TEST_ASSERT_EQUAL_HEX8(0x03,fake::imu_registers[0x41]);
    TEST_ASSERT_EQUAL_HEX8(0x01,fake::imu_registers[0x43]);
}
void reversedVehicleDirectionKeepsFeedbackConsistent()
{
    for(const int direction : {1,-1}) {
        resetMotor();ErrorInfo error{};TEST_ASSERT_EQUAL(ESP_OK,motor::initialize(&error));
        motor::WheelState wheels{};TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheels,&error));
        // 车辆前进对应原始编码器减小；跨零点运动也必须得到正轮速。
        fake::encoder_count=direction>0 ? 4090 : 6;fake::now+=1000;
        TEST_ASSERT_EQUAL(ESP_OK,motor::readWheelState(&wheels,&error));
        TEST_ASSERT_TRUE(wheels.left_velocity_rad_s*direction>0);
        TEST_ASSERT_TRUE(wheels.right_velocity_rad_s*direction>0);
        // 电角度仍跟随对齐后的编码器方向，不能跟随车辆坐标一起取反。
        TEST_ASSERT_TRUE(motor::left_state.electrical_angle_rad*direction<0);
        TEST_ASSERT_TRUE(motor::right_state.electrical_angle_rad*direction<0);
        fake::mv[1]=fake::mv[3]=1650-25*direction;
        motor::CurrentFeedback feedback{};
        TEST_ASSERT_EQUAL(ESP_OK,motor::runCurrentControl({0.2f*direction,0.2f*direction},&feedback,&error));
        TEST_ASSERT_TRUE(feedback.valid);
        TEST_ASSERT_TRUE(motor::left_state.pi.integral_v*direction<0);
        TEST_ASSERT_TRUE(motor::right_state.pi.integral_v*direction<0);
        TEST_ASSERT_TRUE(feedback.left.iq_measured_a*direction>0);
        TEST_ASSERT_TRUE(feedback.right.iq_measured_a*direction>0);
        TEST_ASSERT_TRUE(feedback.left.uq_applied_v*direction>0);
        TEST_ASSERT_TRUE(feedback.right.uq_applied_v*direction>0);
    }
}
extern "C" void app_main()
{
    UNITY_BEGIN();
    RUN_TEST(statusCompatibility);RUN_TEST(replayRetriesAndReconnect);RUN_TEST(errorsAndRing);RUN_TEST(wirePackets);RUN_TEST(gates);
    RUN_TEST(adcInitializationFailures);RUN_TEST(adcRuntimeFailures);RUN_TEST(offsets);
    RUN_TEST(motorFailures);RUN_TEST(failedSamplesDoNotCommit);
    RUN_TEST(outputAgeStages);RUN_TEST(outputAgeBoundary);RUN_TEST(timingSnapshotsAndFirstRelease);
    RUN_TEST(currentOutputAgeStages);RUN_TEST(currentOutputAgeBoundary);RUN_TEST(readBudgetsRemainTwoMilliseconds);
    RUN_TEST(independentBalanceWithoutArm);RUN_TEST(independentBalanceCurrentPath);
    RUN_TEST(balanceStopRequiresNewArm);RUN_TEST(balanceStopSurvivesUnobservedReconnect);RUN_TEST(firstAttitudeUsesMeasuredTilt);
    RUN_TEST(attitudeFilterUsesActualInterval);RUN_TEST(outerLoopAtTwoHundredHzAttitude);RUN_TEST(imuConfiguresBothSensorsAtEightHundredHz);
    RUN_TEST(reversedVehicleDirectionKeepsFeedbackConsistent);
    if (UNITY_END()==0) { puts("DIAGNOSTICS_TESTS_PASS"); }
}
