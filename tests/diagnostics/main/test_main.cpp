#include "unity.h"
#include "diagnostic_store.hpp"
#include "ble_status_codec.hpp"
#include "remote_protocol_checks.cpp"
#include "../../../components/BSP/Motor/current_sense.cpp"
#include "../../../components/BSP/Motor/motor_foc_service.cpp"
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
extern "C" void app_main()
{
    UNITY_BEGIN();
    RUN_TEST(statusCompatibility);RUN_TEST(replayRetriesAndReconnect);RUN_TEST(errorsAndRing);RUN_TEST(wirePackets);RUN_TEST(gates);
    RUN_TEST(adcInitializationFailures);RUN_TEST(adcRuntimeFailures);RUN_TEST(offsets);
    RUN_TEST(motorFailures);RUN_TEST(failedSamplesDoNotCommit);
    if (UNITY_END()==0) { puts("DIAGNOSTICS_TESTS_PASS"); }
}
