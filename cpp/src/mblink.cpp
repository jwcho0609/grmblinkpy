/**
MIT License (modified)

Copyright (c) 2021 Ghost Robotics
Authors:
Avik De <avik@ghostrobotics.io>
Tom Jacobs <tom.jacobs@ghostrobotics.io>
*/

#include <mblink/mblink.hpp>

#include <set>
#include <string>
#include <iomanip>
#include <iostream>
#include <fstream>   // isWSL check
#include <vector>
#include <chrono>

// Networking helpers for non-Windows
#ifndef _WIN32
  #include <ifaddrs.h>
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
#endif

// core locking / scheduling
#include <pthread.h>
#include <sys/time.h>
#include <sys/resource.h>

#if defined(__linux__)
  #include <sched.h>              // sched_setscheduler, SCHED_FIFO
  #define HAS_RT_SCHED     1
  #define HAS_CPU_AFFINITY 1
#else
  // macOS / Windows: no Linux RT or CPU-affinity APIs
  #define HAS_RT_SCHED     0
  #define HAS_CPU_AFFINITY 0
#endif

// Using
using std::string;
using std::set;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

// rx/tx info
constexpr int rx_tx_core = 0;          // core for rx/tx threads (Linux only)
constexpr int THREAD_PRIORITY = 98;    // RT prio (Linux only)
constexpr int NICENESS = -20;          // niceness (both)

// save thread info
constexpr int save_thread_core = 0;
constexpr int THREAD_PRIORITY_SAVE_THREAD = 80;
constexpr int NICENESS_SAVE_THREAD = 0;

// logging
constexpr int NUM_LATENCY_SAMPLES = 5000;   // Number of samples to track
constexpr int LOOP_PERIOD_US = 1000;        // 1 kHz loop target

std::chrono::steady_clock::time_point rx_thread_loop_start_time = std::chrono::steady_clock::now();
std::vector<std::chrono::microseconds> latencies_rx_thread_;
std::chrono::microseconds total_latency_rx_thread_ = std::chrono::microseconds(0);
std::chrono::microseconds max_latency_rx_thread_   = std::chrono::microseconds(0);

static int count_5us_rx_thread   = 0;
static int count_10us_rx_thread  = 0;
static int count_25us_rx_thread  = 0;
static int count_50us_rx_thread  = 0;
static int count_100us_rx_thread = 0;

namespace gr
{

// Forward decl
set<string> getIPs(bool print = false);

static bool isWSL()
{
#if defined(__linux__)
  // Detect WSL by reading osrelease and searching for "icrosoft"
  std::ifstream osrelease("/proc/sys/kernel/osrelease");
  std::string str_name((std::istreambuf_iterator<char>(osrelease)), std::istreambuf_iterator<char>());
  if (str_name.find("icrosoft") != std::string::npos) {
    return true;
  }
#endif
  return false;
}

int MBLink::queueMessage(uint16_t /*messageLength*/)
{
  int payload_length = mavlink_msg_to_send_buffer(buffer, &msg);

  // Send
  int send_length = sendto(txSocket, (char*)buffer, payload_length, 0,
                           (struct sockaddr*)&mbAddr, sizeof(mbAddr));
  if (send_length <= 0)
    printf("**** Error sending message to robot.\n");

  return (send_length == payload_length);
}

bool MBLink::ipCheck()
{
#ifndef _WIN32
  // Check IP address. You won't send any UDP packets if you don't have an IP on the same subnet.
  bool okIP = false;
  set<string> IPs = getIPs();
  for (const auto& IP : IPs) {
    if (IP.find("192.168.168.") != string::npos) {
      okIP = true;
      break;
    }
  }
  if (!okIP) {
    printf("\n");
    printf("**** Error: No valid IP address, so we can't send packets to robot.\n");
    printf("Check your network adaptor is connected to the robot.\n");
    printf("Configure it to DHCP, or static IP: 192.168.168.100.\n");
    printf("Current IP addresses:\n");
    getIPs(true);
    printf("\n");
    return false;
  }
#endif
  return true;
}

int MBLink::start(bool sim, bool verbose, uint16_t rx_port_alt)
{
  // if (!(sim || ipCheck())) return -1;

  std::cout << "Not Broken 0" << std::endl;
  bool isWin32 = false;
#ifdef _WIN32
  isWin32 = true;
#endif
#if defined(__linux__)
  isWin32 = isWSL();
#endif
  std::cout << isWin32 << std::endl;
  if (!isWin32) {
    MB_ADDR = BCAST_ADDR;
  }

  std::string rxAddr = sim ? LOCAL_ADDR : MB_ADDR;
  std::string txAddr = sim ? LOCAL_ADDR : BCAST_ADDR;

  // Create TX and RX sockets
  txSocket = createSocket(txAddr.c_str(), TX_PORT, true, false, &mbAddr);
  rxSocket = createSocket(rxAddr.c_str(), rx_port_alt > 0 ? rx_port_alt : RX_PORT, false, true, nullptr);

  this->verbose = verbose;
  return 0;
}

void MBLink::rxstart()
{
  std::cout << std::setprecision(2) << std::fixed;
  rxThread = std::thread(&MBLink::rxThreadTask, this);

  // RT scheduling (Linux only)
#if HAS_RT_SCHED
  {
    struct sched_param param {};
    param.sched_priority = THREAD_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
      std::cerr << "RXSTART: Failed to set RT FIFO " << THREAD_PRIORITY << std::endl;
    } else {
      std::cout << "RXSTART: RT FIFO priority " << THREAD_PRIORITY << std::endl;
    }
  }
#else
  std::cout << "RXSTART: RT scheduling not supported on this platform.\n";
#endif

  // Pin to core (Linux only)
#if HAS_CPU_AFFINITY
  {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(rx_tx_core, &cpu_set);
    if (pthread_setaffinity_np(rxThread.native_handle(), sizeof(cpu_set), &cpu_set) != 0) {
      std::cerr << "RXSTART: Failed to set rxThread affinity to core " << rx_tx_core << std::endl;
    } else {
      std::cout << "RXSTART: rxThread pinned to core " << rx_tx_core << std::endl;
    }
  }
#else
  std::cout << "RXSTART: CPU affinity not supported on this platform.\n";
#endif

  if (setpriority(PRIO_PROCESS, 0, NICENESS) == -1) {
    std::cerr << "RXSTART: Failed to set thread niceness!" << std::endl;
  } else {
    std::cout << "RXSTART: Set thread niceness to " << NICENESS << std::endl;
  }
}

void MBLink::rxContinuousStart(std::string filename)
{
  if (!filename.empty())
    filename_ = filename;

  std::cout << std::setprecision(2) << std::fixed;
  std::cout << "rxContinuous started" << std::endl;

  rx_continuous_active_flag_ = true;
  saving = true;
  rxThread   = std::thread(&MBLink::rxThreadTask, this);
  saveThread = std::thread(&MBLink::saveThreadTask, this);

  // Affinity (Linux only)
#if HAS_CPU_AFFINITY
  {
    cpu_set_t cpu_set_rx_thread;
    CPU_ZERO(&cpu_set_rx_thread);
    CPU_SET(rx_tx_core, &cpu_set_rx_thread);
    if (pthread_setaffinity_np(rxThread.native_handle(), sizeof(cpu_set_rx_thread), &cpu_set_rx_thread) != 0) {
      std::cerr << "rxContinuousStart: Failed to set rxThread affinity to core " << rx_tx_core << std::endl;
    } else {
      std::cout << "rxContinuousStart: rxThread pinned to core " << rx_tx_core << std::endl;
    }

    cpu_set_t cpu_set_saveThread;
    CPU_ZERO(&cpu_set_saveThread);
    CPU_SET(save_thread_core, &cpu_set_saveThread);
    if (pthread_setaffinity_np(saveThread.native_handle(), sizeof(cpu_set_saveThread), &cpu_set_saveThread) != 0) {
      std::cerr << "rxContinuousStart: Failed to set saveThread affinity to core " << save_thread_core << std::endl;
    } else {
      std::cout << "rxContinuousStart: saveThread pinned to core " << save_thread_core << std::endl;
    }
  }
#else
  std::cout << "rxContinuousStart: CPU affinity not supported on this platform.\n";
#endif
}

RxAccumData_t MBLink::rxstop(bool waitForExit)
{
  rxKeepRunning.exchange(false);

  if (waitForExit) {
    if (rxThread.joinable())   rxThread.join();
    if (saveThread.joinable()) saveThread.join();
  }
  std::cout << "MBLink rxstop completed.\n";
  RxAccumData_t temp = std::move(rx_accum_);
  return temp;
}

RxAccumData_t MBLink::rxContinuousStop(bool waitForExit)
{
  rxKeepRunning.exchange(false);
  saving = false;

  if (waitForExit) {
    if (rxThread.joinable())   rxThread.join();
    if (saveThread.joinable()) saveThread.join();
  }
  std::cout << "MBLink rxstopcontinuous completed.\n";
  return rx_accum_;
}

void MBLink::rxThreadTask()
{
  char buf[10000];
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  int addrlen = sizeof(addr);

  float t0 = 0, avgdt = 0;
  steady_clock::time_point lastPrint = steady_clock::now();

  // Timeout to detect if the receive thread will hang
  tv.tv_sec = 1;
  tv.tv_usec = 0;

  // RT scheduling (Linux only)
#if HAS_RT_SCHED
  {
    struct sched_param param {};
    param.sched_priority = THREAD_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
      std::cerr << "rxThreadTask: Failed to set RT FIFO " << THREAD_PRIORITY << std::endl;
    } else {
      std::cout << "rxThreadTask: RT FIFO priority " << THREAD_PRIORITY << std::endl;
    }
  }
#else
  std::cout << "rxThreadTask: RT scheduling not supported on this platform.\n";
#endif

  if (setpriority(PRIO_PROCESS, 0, NICENESS) == -1) {
    std::cerr << "rxThreadTask: Failed to set niceness!" << std::endl;
  } else {
    std::cout << "rxThreadTask: Set niceness to " << NICENESS << std::endl;
  }

#ifdef _WIN32
  fd_set fds;
  int n;
  FD_ZERO(&fds);
  FD_SET(rxSocket, &fds);
#endif

  while (rxKeepRunning)
  {
#ifdef _WIN32
    // Detect mb disconnection (SO_RCVTIMEO replacement for win32)
    n = select((int)rxSocket, &fds, NULL, NULL, &tv);
    if (n <= 0) {
      std::cerr << "MBLink::rxThreadTask recvfrom error (timeout).\n";
      rxKeepRunning.exchange(false);
      continue;
    }
#endif

    // blocking recvfrom
    int recvlen = recvfrom(rxSocket, buf, sizeof(buf), 0,
                           (struct sockaddr*)&addr, (socklen_t*)&addrlen);

    if (recvlen <= 0) {
      std::cerr << "[MBLink::rxThreadTask] recvfrom error (timeout)." << std::endl;
      rxKeepRunning.exchange(false);
      continue;
    }

    constexpr uint32_t start_word = 0x87654321;
    int numparsed = 0;
    if (memcmp(&buf[0], &start_word, 4) == 0) {
      newPlannerUDPMsg(&buf[4], recvlen - 4);
    } else {
      conditionMutex.lock();
      if (rx_continuous_active_flag_) {
        numparsed = parseContinuous(buf, recvlen, pp_buffer_, saving);
      } else {
        numparsed = parse(buf, recvlen); // ignore high-rate messages
      }
      conditionMutex.unlock();
    }

    float t1 = rxdata["y"][rxdata["y"].size() - 1];
    if (numparsed > 0 && t > 1e-3f) {
      avgdt += 0.1f * (t1 - t0 - avgdt);
      this->avgRate = 1 / avgdt;
      t0 = t1;
    }

    if (verbose) {
      auto now = steady_clock::now();
      bool pl_running = t - last_pl_time_ < 0.5f && ctrldata_["pl_y"].size() > 0;
      int print_interval = pl_running ? 5000 : 10000;
      if (duration_cast<std::chrono::milliseconds>(now - lastPrint).count() > print_interval) {
        std::cout << "[mblink rx]\tt=" << t0 << "\trate=" << 1 / avgdt;
        if (pl_running) {
          std::cout << "\tq=" << ctrldata_["pl_y"].head<6>().transpose()
                    << "\tu=" << ctrldata_["u"].head<3>().transpose()
                    << "\tgoal=" << ctrldata_["goal"].transpose()
                    << "\tplanner_cmd=" << mmgr.get(Behav_PLANNER_CMD);
        } else {
          std::cout << "\tstatus=" << rxdata["behavior"][2]
                    << "\tvoltage(batt,mot)=" << rxdata["voltage"].transpose();
        }
        std::cout << std::endl;
        lastPrint = now;
      }

      // Loop latency bookkeeping
      auto loop_end_time  = std::chrono::steady_clock::now();
      auto total_loop_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_end_time - rx_thread_loop_start_time);
      auto actual_latency  = std::chrono::microseconds(std::abs(total_loop_time.count() - LOOP_PERIOD_US));

      latencies_rx_thread_.push_back(actual_latency);
      total_latency_rx_thread_ += actual_latency;
      if (actual_latency > max_latency_rx_thread_) max_latency_rx_thread_ = actual_latency;

      if (actual_latency > std::chrono::microseconds(100)) count_100us_rx_thread++;
      if (actual_latency > std::chrono::microseconds(50))  count_50us_rx_thread++;
      if (actual_latency > std::chrono::microseconds(25))  count_25us_rx_thread++;
      if (actual_latency > std::chrono::microseconds(10))  count_10us_rx_thread++;
      if (actual_latency > std::chrono::microseconds(5))   count_5us_rx_thread++;

      if (latencies_rx_thread_.size() == NUM_LATENCY_SAMPLES) {
        auto avg_latency = total_latency_rx_thread_ / static_cast<int>(latencies_rx_thread_.size());
        std::cout << "Average Update Loop Latency for rxThreadTask (last "
                  << latencies_rx_thread_.size() << "): " << avg_latency.count()
                  << " us, Max: " << max_latency_rx_thread_.count()
                  << " us, Percent over (5,10,25,50,100 us): "
                  << std::fixed << std::setprecision(2)
                  << (100.0 * count_5us_rx_thread)   / NUM_LATENCY_SAMPLES << ", "
                  << (100.0 * count_10us_rx_thread)  / NUM_LATENCY_SAMPLES << ", "
                  << (100.0 * count_25us_rx_thread)  / NUM_LATENCY_SAMPLES << ", "
                  << (100.0 * count_50us_rx_thread)  / NUM_LATENCY_SAMPLES << ", "
                  << (100.0 * count_100us_rx_thread) / NUM_LATENCY_SAMPLES
                  << std::endl;

        latencies_rx_thread_.clear();
        total_latency_rx_thread_ = std::chrono::microseconds(0);
        max_latency_rx_thread_   = std::chrono::microseconds(0);
        count_5us_rx_thread = count_10us_rx_thread = count_25us_rx_thread =
        count_50us_rx_thread = count_100us_rx_thread = 0;
      }
      rx_thread_loop_start_time = std::chrono::steady_clock::now();
    }
  }

  std::unique_lock<std::mutex> lk(conditionMutex);
  cv.notify_one(); // so that we can quit
  std::cout << "NO" << std::endl;
}

void MBLink::saveThreadTask()
{
  // RT scheduling (Linux only)
#if HAS_RT_SCHED
  {
    struct sched_param param {};
    param.sched_priority = THREAD_PRIORITY_SAVE_THREAD;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
      std::cerr << "saveThreadTask: Failed to set RT FIFO " << THREAD_PRIORITY_SAVE_THREAD << std::endl;
    } else {
      std::cout << "saveThreadTask: RT FIFO priority " << THREAD_PRIORITY_SAVE_THREAD << std::endl;
    }
  }
#else
  std::cout << "saveThreadTask: RT scheduling not supported on this platform.\n";
#endif

  if (setpriority(PRIO_PROCESS, 0, NICENESS_SAVE_THREAD) == -1) {
    std::cerr << "saveThreadTask: Failed to set niceness!" << std::endl;
  } else {
    std::cout << "saveThreadTask: Set niceness to " << NICENESS_SAVE_THREAD << std::endl;
  }

  while (saving)
  {
    auto lock = pp_buffer_.getUniqueLock();
    pp_buffer_.getCV().wait(lock, [this]{ return pp_buffer_.needsSavingQ(); });

    saveBucket(*(pp_buffer_.getOtherBuffer()));
    pp_buffer_.beenSaved();
  }

  std::cout << "KILL DETECTED: SAVING DATA" << std::endl;
  if (!pp_buffer_.getOtherBuffer()->isFull()) {
    saveBucket(*pp_buffer_.getOtherBuffer());
    pp_buffer_.getOtherBuffer()->clear();
  }
  std::cout << "KILL DETECTED: DATA SAVED!" << std::endl;
}

void MBLink::saveBucket(const Buffer &bucket)
{
  std::ofstream outfile(filename_, std::ios::binary | std::ios::app);
  auto data = bucket.getDataCopy();
  outfile.write(data.data(), data.size());
  outfile.close();
}

float MBLink::readParam(const std::string &name, bool print_on_failure)
{
  float retval = 0;
  std::chrono::milliseconds timeout(10);
  requestParam(name);
  std::unique_lock<std::mutex> lk(paramMutex);

  if (paramcv.wait_for(lk, timeout, [this]{ return rxdata["param_value"].size() > 0; })) {
    retval = rxdata["param_value"][0];
  } else if (print_on_failure) {
    std::cerr << "[MBLink::readParam] did not receive param " << name << std::endl;
  }
  return retval;
}

void MBLink::setRetry(const std::string &name, float val)
{
  std::chrono::milliseconds paramwait(20);
  for (int i = 0; i < 10; ++i) {
    setParam(name, val);
    std::this_thread::sleep_for(paramwait);
    if (std::abs(readParam(name, false) - val) < 1e-4f) {
      std::cout << name << " = " << val << std::endl;
      return;
    }
  }
  std::cerr << "[MBLink::setRetry] error: " << name << " not set" << std::endl;
}

Eigen::VectorXf MBLink::readWritePlannerParam(const std::string &name, const Eigen::VectorXf &val, bool write)
{
  Eigen::VectorXf retval;
  std::chrono::milliseconds timeout(20);

  // clear param value (can check size for reception)
  paramMutex.lock();
  rxdata["param_value"].resize(0);
  paramMutex.unlock();

  // Send the mavlink message
  sendPlannerParam(name, val, write);

  // Wait for response
  std::unique_lock<std::mutex> lk(paramMutex, std::defer_lock);
  lk.lock();

  if (paramcv.wait_for(lk, timeout, [this]{ return rxdata["param_value"].size() > 0; })) {
    retval = rxdata["param_value"];
    if (retval.isApprox(val)) {
      std::cout << name << " = " << retval.transpose() << std::endl;
    } else if (write) {
      std::cerr << "[MBLink::readWritePlannerParam] Received " << name << " = "
                << retval.transpose() << " instead of " << val.transpose() << std::endl;
    }
  } else {
    std::cerr << "[MBLink::readWritePlannerParam] did not receive param " << name << std::endl;
  }
  return retval;
}

bool MBLink::ensureMode(const std::string &fieldName, uint32_t valdes, uint32_t timeoutMS)
{
  const uint32_t modewait = 50;
  std::chrono::milliseconds timeout(modewait);

  auto tosend = mmgr.set(fieldName, valdes);
  for (size_t i = 0; i < timeoutMS / modewait; ++i) {
    setModeRaw(std::get<0>(tosend), std::get<1>(tosend));
    std::unique_lock<std::mutex> lk(modeMutex);
    if (modecv.wait_for(lk, timeout, [this, fieldName, valdes]{ return mmgr.get(fieldName) == valdes; }))
      return true;
  }
  return false;
}

void MBLink::selfCheck(int16_t action, int16_t param)
{
  auto tosend = mmgr.selfCheck(action, param);
  setModeRaw(std::get<0>(tosend), std::get<1>(tosend));
}

RxData_t MBLink::get()
{
  static RxData_t ret;
  std::unique_lock<std::mutex> lk(conditionMutex);
  cv.wait(lk, [this]{ return newRxData.load() || !rxKeepRunning.load(); });
  if (rxKeepRunning) {
    newRxData.exchange(false);
    ret = rxdata; // copy assignment
  }
  return ret;
}

#ifndef _WIN32
// Return a list of IP addresses on system
set<string> getIPs(bool print)
{
  set<string> IPs;
  struct ifaddrs *ifap, *ifa;
  struct sockaddr_in *sa;
  char *addr;

  if (getifaddrs(&ifap) != 0) return IPs;
  for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
    if (!ifa || !ifa->ifa_addr) continue;
    if (ifa->ifa_addr->sa_family == AF_INET) {
      sa = (struct sockaddr_in *)ifa->ifa_addr;
      addr = inet_ntoa(sa->sin_addr);
      IPs.insert(addr);
      if (print)
        printf("Address: %s\t\tInterface: %s\n", addr, ifa->ifa_name);
    }
  }
  freeifaddrs(ifap);
  return IPs;
}
#endif

} // namespace gr
