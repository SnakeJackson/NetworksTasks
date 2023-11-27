/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/dvhop-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <fstream>


using namespace ns3;

/**
 * \brief Test script.
 *
 * This script creates 1-dimensional grid topology and then ping last node from the first one:
 *
 * [10.0.0.1] <-- step --> [10.0.0.2] <-- step --> [10.0.0.3] <-- step --> [10.0.0.4]
 *
 *
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <vector>
using namespace std;

struct point
{
    double x,y;
};

float norm (point p) // get the norm of a vector
{
    return pow(pow(p.x,2)+pow(p.y,2),.5);
}

point trilateration(point point1, point point2, point point3, double r1, double r2, double r3) {
    // calculate differences between the coordinates
    point resultPose;
    double x1 = point1.x, y1 = point1.y;
    double x2 = point2.x, y2 = point2.y;
    double x3 = point3.x, y3 = point3.y;

    double d1 = r1 * r1 - r2 * r2 - x1 * x1 - y1 * y1 + x2 * x2 + y2 * y2;
    double d2 = r2 * r2 - r3 * r3 - x2 * x2 - y2 * y2 + x3 * x3 + y3 * y3;

    // calculate the coordinates using trilateration formula
    double a = 2 * (x2 - x1);
    double b = 2 * (y2 - y1);
    double c = 2 * (x3 - x2);
    double d = 2 * (y3 - y2);

    double det = a * d - b * c;

    if (det == 0) {
        // The points are collinear, trilateration is not possible
        resultPose.x = resultPose.y = NAN;
    } else {
        resultPose.x = (d1 * d - b * d2) / det;
        resultPose.y = (a * d2 - d1 * c) / det;
    }
    // defined length estimation
    return resultPose;
}



class DVHopExample
{
public:
  DVHopExample ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);
  int getBeacons () {return beacons;}
  int getSeed () {return seed;}
  //int getNodesKilled () {return nodesKilled;}
  std::vector<int>*  getBeaconCoords() {return &beaconCoords;}
  std::vector<float> getHopSizes() {return hopSizes;}
  // std::map<Ipv4Address, ns3::dvhop::BeaconInfo> GetDistanceTable(int nodeIndex) const;
/*
  int KillCalculator ();
  double KillTimeCalculator ();
  void NodeKiller ();
*/


private:
  ///\name parameters
  //\{
  /// Number of nodes
  uint32_t size;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;

  bool randomSeed;

  float beaconPercent;

  //\}
  int beacons;
  std::vector<int> beaconCoords;
  std::vector<float> hopSizes;

  int nodesToKill;

  int seed = 25565;

  ///\name network
  //\{
  NodeContainer nodes;
  NetDeviceContainer devices;
  Ipv4InterfaceContainer interfaces;
  //\}



private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
  void CreateBeacons();
  void CalculateHopSize();
  void CalculateCoordinates();
};

void testPrint(){std::cout << "TESTING----------------------------------------" << std::endl;}

int main (int argc, char **argv)
{
  DVHopExample test;
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");


  test.Run ();
  test.Report (std::cout);



  //Debugging Reports----------------------------------
  //Beacon count
  std::cout << "Beacons: " << test.getBeacons() << std::endl;
  //std::cout << "Nodes Killed: " << test.getNodesKilled() << std::endl;
  //Beacon Locations
  int bcount = 0;
  int itr = 0;
  for(int x = (test.getBeaconCoords()->size())/2; x > 0; x--){
    std::cout << "Beacon " << bcount << " is at " << test.getBeaconCoords()->at(itr) << " " << test.getBeaconCoords()->at(itr+1) << std::endl;
    bcount++;
    itr += 2;
  }
  //Seed #
  if(test.getSeed() == 25565)
    std::cout << "Seed: " << test.getSeed() << ". Not random." << std::endl;
  else
    std::cout << "Seed: " << test.getSeed() << ". Is random." << std::endl;
  return 0;

  std::vector<float> hopSizes = test.getHopSizes();
  for(int x = 0; x < std::ceil(hopSizes.size()); x++){
  	std::cout << hopSizes[x] << std::endl;
  }

}

//-----------------------------------------------------------------------------
DVHopExample::DVHopExample () :
  size (1000),
  step (100),
  totalTime (10),
  pcap (true),
  printRoutes (true),
  randomSeed (false),
  beaconPercent (.2)
{
}

bool
DVHopExample::Configure (int argc, char **argv)
{
  // Enable DVHop logs by default. Comment this if too noisy
  //LogComponentEnable("DVHopRoutingProtocol", LOG_LEVEL_ALL);


  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);
  cmd.AddValue ("random", "Randomize seed", randomSeed);
  cmd.AddValue ("beaconPercent", "Set percentage of beacons", beaconPercent);

  cmd.Parse (argc, argv);

  if(randomSeed)
    seed = rand() % (9999-1000 + 1) + 1000;

  SeedManager::SetSeed (seed);
  return true;
}

// Handler for node death
void
KillNode(int node_num, double death_time, Ptr<dvhop::RoutingProtocol> node_dvhop) {
  // Kill node by moving it far away
  // std::cout << "Killed node " << node_num << " at " << death_time << std::endl;
  // Ptr<ConstantPositionMobilityModel> mob = node_ptr->GetObject<ConstantPositionMobilityModel>();
  // Vector new_position = Vector(0.0, 20000.0 + 1000*node_num, 0.0);
  // mob->SetPosition(new_position);
  node_dvhop->KillNode();
}

void
DVHopExample::Run ()
{
  //Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  CreateBeacons();


  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  Simulator::Stop (Seconds (totalTime));

  AnimationInterface anim("animation.xml");

  //Change beacons colors
  int nodeitr = 0;
  Ptr<Node> node;
  for(int x = beacons; x > 0; x--){
    node = nodes.Get(nodeitr);
    anim.UpdateNodeColor(node->GetId(), 0, 0, 255);
    ++nodeitr;
  }
  
  // Setup random number generator for Node Death times
  Ptr<UniformRandomVariable> time_generator = CreateObject<UniformRandomVariable>();
  time_generator->SetAttribute("Min", DoubleValue(0));
  time_generator->SetAttribute("Max", DoubleValue(totalTime));

  double death_time;
  Ptr<ConstantPositionMobilityModel> mob;
  Ptr<Node> current_node;
  // Iterate through each node to schedule a death time
  for(uint32_t node_num = 0; node_num < size; node_num++)
  {
    current_node = nodes.Get(node_num);
    // mob = nodes.Get(node_num)->GetObject<ConstantPositionMobilityModel>();
    Ptr<Ipv4RoutingProtocol> nodeProto = current_node -> GetObject<Ipv4>() -> GetRoutingProtocol ();
	  Ptr<dvhop::RoutingProtocol> node_dvhop = DynamicCast<dvhop::RoutingProtocol> (nodeProto);
    death_time = time_generator->GetValue();
    // Simulator::Schedule(Seconds(death_time), &KillNode, node_num, death_time, current_node);
    Simulator::Schedule(Seconds(death_time), &KillNode, node_num, death_time, node_dvhop);
  }

  Simulator::Run ();

  CalculateHopSize();

  Simulator::Destroy ();
}

void
DVHopExample::Report (std::ostream &)
{
}

void
DVHopExample::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
    {
      std::ostringstream os;
      os << "node-" << i;
      std::cout << "Creating node: "<< os.str ()<< std::endl ;
      Names::Add (os.str (), nodes.Get (i));
    }

  Ptr<UniformRandomVariable> xs = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> ys = CreateObject<UniformRandomVariable> ();
  xs->SetAttribute("Max", DoubleValue(100));
  ys->SetAttribute("Max", DoubleValue(100));

  Ptr<ns3::RandomRectanglePositionAllocator> allocator = CreateObject<ns3::RandomRectanglePositionAllocator> ();
  allocator -> SetX(xs);
  allocator -> SetY(ys);

  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator(allocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel", "");
  mobility.Install (nodes);
}

void
DVHopExample::CreateBeacons ()
{

beacons = 0;
NodeContainer::Iterator i = nodes.Begin();
if(size > 30){
 for(int x = 0; x < std::ceil((size)*beaconPercent); x++){
	Ptr<Ipv4RoutingProtocol> proto = nodes.Get (x)->GetObject<Ipv4>()->GetRoutingProtocol ();
 	Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol> (proto);
	dvhop->SetIsBeacon(true);

  //Tell dvhop algorithim real location
  Ptr<Node> node = *i;
  Ptr<MobilityModel> nodeMB = node->GetObject<MobilityModel> ();
  if (!nodeMB)
    continue;
  Vector actualPosition = nodeMB->GetPosition();
  dvhop->SetPosition(actualPosition.x, actualPosition.y);
  beaconCoords.push_back(actualPosition.x);
  beaconCoords.push_back(actualPosition.y);
	beacons++;
  ++i;
  }
}
else{
  for(int x = 0; x < 3; x++){
    Ptr<Ipv4RoutingProtocol> proto = nodes.Get (x)->GetObject<Ipv4>()->GetRoutingProtocol ();
    Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol> (proto);
    dvhop->SetIsBeacon (true);
    Ptr<Node> node = *i;
    Ptr<MobilityModel> nodeMB = node->GetObject<MobilityModel> ();
    if (!nodeMB)
      continue;
    Vector actualPosition = nodeMB->GetPosition();
  	dvhop->SetPosition(actualPosition.x, actualPosition.y);
    beaconCoords.push_back(actualPosition.x);
    beaconCoords.push_back(actualPosition.y);
  	beacons++;
    ++i;
  }
}
}

void DVHopExample::CalculateHopSize (){
  std::map<int, float> hopsize;

  std::ofstream outfile;
  outfile.open("data.csv");

  for(int x = 0; x < beacons; x++){
  	Ptr<Ipv4RoutingProtocol> nodeProto = nodes.Get(x) -> GetObject<Ipv4>() -> GetRoutingProtocol ();
	Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol> (nodeProto);
	ns3::dvhop::DistanceTable table = dvhop -> GetDistanceTable();
	std::map<Ipv4Address, dvhop::BeaconInfo> inner = table.Inner();

	double x1 = dvhop -> GetXPosition();
	double y1 = dvhop -> GetYPosition();

	int hops = 0;
	double sum = 0;

	for(const auto& kv : inner){
		ns3::dvhop::BeaconInfo info = kv.second;

		double x2 = info.GetPosition().first;
		double y2 = info.GetPosition().second;

		sum += sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
		hops += info.GetHops();
	}

	//Ipv4Address ipv4 = (dvhop -> GetIpv4() -> GetAddress(0, 0)).GetAddress();

	if (hops == 0)
		hopsize[x];
	else
		hopsize[x] = sum/hops;

	std::cout << hopsize[x] << std::endl;

  }


  NodeContainer::Iterator itr = nodes.Begin() + beacons;

  for(int i = beacons; i < std::ceil(size); i++){
  	Ptr<Ipv4RoutingProtocol> proto = nodes.Get(i) -> GetObject<Ipv4>() -> GetRoutingProtocol();
	Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol> (proto);
	ns3::dvhop::DistanceTable table = dvhop -> GetDistanceTable();
	std::map<Ipv4Address, ns3::dvhop::BeaconInfo> inner = table.Inner();


	if(inner.size() >= 3){

		auto a = inner.begin();
      		double xa = a->second.GetPosition().first;
      		double ya = a->second.GetPosition().second;

      		// hops size * hops to node

      		double ra = hopsize[(a -> first).Get() - 167772161] * a->second.GetHops();

      		auto b = inner.begin();
		b++;
      		double xb = b->second.GetPosition().first;
      		double yb = b->second.GetPosition().second;

      		// hops size * hops to node
      		double rb = hopsize[(b -> first).Get() - 167772161] * b->second.GetHops();

      		auto c = inner.begin();
		c++;
		c++;
      		double xc = c->second.GetPosition().first;
      		double yc = c->second.GetPosition().second;

      		// hops size * hops to node
      		double rc = hopsize[(c -> first).Get() - 167772161] * c->second.GetHops();

		/*
		std::cout << (a -> first).Get() << std::endl;
		std::cout << a->second.GetHops() << b->second.GetHops() << c->second.GetHops() << std::endl;
		std::cout << "Beacon1 (x, y): (" << xa << ", " << ya << ") " << "Beacon2 (x, y): (" << xb << ", " << yb << ") " << "Beacon3 (x, y): (" << xc << ", " << yc << ") " <<std::endl;
	        std::cout << "Beacon1 r: " << ra << " Beacon2 r: " << rb << " Beacon3 r: " << rc << std::endl;
		*/

		point aPoint;
		aPoint.x = xa;
		aPoint.y = ya;

		point bPoint;
                bPoint.x = xb;
                bPoint.y = yb;

		point cPoint;
                cPoint.x = xc;
                cPoint.y = yc;

		point nodeGuess = trilateration(aPoint, bPoint, cPoint, ra, rb, rc);

		std::cout << "NODE " << i << " GUESS: "  << nodeGuess.x << " " << nodeGuess.y << std::endl;


    //Get actual node position
     Ptr<Node> node = *itr;
     Ptr<MobilityModel> nodeMB = node->GetObject<MobilityModel> ();
     if (!nodeMB)
       continue;
     Vector actualPosition = nodeMB->GetPosition();
     std::cout << "NODE " << i << " ACTUAL: "  << actualPosition.x << " " << actualPosition.y << std::endl;
     ++itr;

     //Write to csv file

     outfile << i << ", " << nodeGuess.x << ", " << nodeGuess.y << ", " << actualPosition.x << ", " << actualPosition.y << std::endl;
	}
  }

}


// std::map<Ipv4Address, ns3::dvhop::BeaconInfo> DVHopExample::GetDistanceTable(int nodeIndex) const
// {
//     Ptr<Ipv4RoutingProtocol> proto = nodes.Get(nodeIndex)->GetObject<Ipv4>()->GetRoutingProtocol();
//     Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol>(proto);
//     ns3::dvhop::DistanceTable table = dvhop->GetDistanceTable();
//     return table.GetDistanceTable();
// }

void
DVHopExample::CreateDevices ()
{
  WifiMacHelper wifiMac = WifiMacHelper ();
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi = WifiHelper ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("aodv"));
    }
}

void
DVHopExample::InstallInternetStack ()
{
  DVHopHelper dvhop;
  // you can configure DVhop attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (dvhop); // has effect on the next Install ()
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  Ptr<OutputStreamWrapper> distStream = Create<OutputStreamWrapper>("dvhop.distances", std::ios::out);
  dvhop.PrintDistanceTableAllAt(Seconds(9), distStream);

  /*
  if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dvhop.routes", std::ios::out);
      dvhop.PrintRoutingTableAllAt (Seconds (8), routingStream);
    }
   */
}
