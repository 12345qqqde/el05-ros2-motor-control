#include "motor_ros2/motor_cfg.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
namespace { std::atomic_bool stop{false}; void sig(int){stop=true;} }
int main(int argc,char **argv){
  bool ok=false, large_motion_confirmed=false; double duration=60, amp=.02, freq=.1, speed=.1;
  std::array<bool,5> active{}; active.fill(true);
  for(int i=1;i<argc;i++){std::string a=argv[i];
    if(a=="--confirm-hardware") ok=true;
    else if(a=="--confirm-large-motion") large_motion_confirmed=true;
    else if(i+1<argc && a=="--duration") duration=std::stod(argv[++i]);
    else if(i+1<argc && a=="--amplitude") amp=std::stod(argv[++i]);
    else if(i+1<argc && a=="--frequency") freq=std::stod(argv[++i]);
    else if(i+1<argc && a=="--speed") speed=std::stod(argv[++i]);
    else if(i+1<argc && a=="--skip-id") {
      int id=std::stoi(argv[++i]);
      if(id<1 || id>5){std::cerr<<"--skip-id must be between 1 and 5.\n"; return 2;}
      active[(size_t)(id-1)]=false;
    }
    else {std::cerr<<"Usage: el05_all_sine --confirm-hardware [--confirm-large-motion] [--skip-id N] [--duration s] [--amplitude rad] [--frequency Hz] [--speed rad/s]\n"; return 2;}}
  constexpr double normal_max_amplitude = .05;
  constexpr double large_motion_max_amplitude = .523599; // 30 degrees
  if(amp > normal_max_amplitude && !large_motion_confirmed){
    std::cerr<<"Large amplitude requires --confirm-large-motion after checking clearance and emergency stop.\n";
    return 2;
  }
  const double max_amp = large_motion_confirmed ? large_motion_max_amplitude : normal_max_amplitude;
  if(!ok||duration<=0||amp<=0||amp>max_amp||freq<=0||freq>.2||speed<=0||speed>.2){
    std::cerr<<"Refusing run: confirmation required; limits amp<="<<max_amp<<" rad, freq<=.2, speed<=.2.\n";
    return 2;
  }
  std::signal(SIGINT,sig); std::signal(SIGTERM,sig);
  std::array<std::unique_ptr<RobStrideMotor>,5> m; std::array<double,5> c{};
  try { for(int i=0;i<5;i++){if(!active[i]) continue; m[i]=std::make_unique<RobStrideMotor>("can0",0xFD,(uint8_t)(i+1),0); m[i]->Get_RobStrite_Motor_parameter(0x7019); c[i]=m[i]->drw.mechPos.data; if(!std::isfinite(c[i])) throw std::runtime_error("invalid position feedback");}
    std::cout<<"Selected motors ready; skipped IDs:"; for(int i=0;i<5;i++) if(!active[i]) std::cout<<" "<<(i+1); std::cout<<"; amplitude="<<amp<<" rad; duration="<<duration<<" s\n";
    auto start=std::chrono::steady_clock::now(); while(!stop){double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(); if(t>=duration)break; double q=amp*std::sin(2*M_PI*freq*t); for(int i=0;i<5;i++) if(active[i]) m[i]->RobStrite_Motor_PosCSP_control((float)speed,(float)(c[i]+q)); std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  } catch(const std::exception&e){std::cerr<<"Stopping all motors: "<<e.what()<<"\n";}
  for(auto &x:m)if(x){try{x->Disenable_Motor(0);}catch(...) {}} std::cout<<"All motors stopped.\n"; return 0;
}
