# This script is created by Peng Feng

#===================================
#     Simulation parameters setup
#===================================
set val(chan)   Channel/WirelessChannel    ;# channel type
set val(prop)   Propagation/TwoRayGround   ;# radio-propagation model
set val(netif)  Phy/WirelessPhy            ;# network interface type
set val(mac)    Mac/CogMOR                 ;# MAC type
set val(ifq)    Queue/DropTail/PriQueue    ;# interface queue type
set val(ll)     LL                         ;# link layer type
set val(ant)    Antenna/OmniAntenna        ;# antenna model
set val(ifqlen) 50                         ;# max packet in ifq
set val(su)     5                          ;# number of su nodes
set val(rp)     AODV                       ;# routing protocol
set val(x)      1000                       ;# X dimension of topography
set val(y)      1000                       ;# Y dimension of topography
set val(stop)   250.0                      ;# time of simulation end
set val(pu)     4                          ;# set the number of PU nodes
set val(nc)     10                         ;# set the number of channels
set val(out)    CogMOR3
#===================================
#        Initialization        
#===================================
#Create a ns simulator
set ns [new Simulator]

#Setup topography object
set topo       [new Topography]
$topo load_flatgrid $val(x) $val(y)
set god [create-god [expr $val(su) + $val(pu)]]

#Open the NS trace file
set tracefile [open $val(out).tr w]
$ns trace-all $tracefile
$ns use-newtrace

#Open the NAM trace file
set namfile [open $val(out).nam w]
$ns namtrace-all $namfile
$ns namtrace-all-wireless $namfile $val(x) $val(y)

#===================================
#        Create wireless channel        
#===================================
$ns add-channel [expr $val(nc)] $val(chan)

#===================================
#     Mobile node parameter setup
#===================================
$ns node-config -adhocRouting   $val(rp) \
                -llType         $val(ll) \
                -macType        $val(mac) \
                -ifqType        $val(ifq) \
                -ifqLen         $val(ifqlen) \
                -antType        $val(ant) \
                -propType       $val(prop) \
                -phyType        $val(netif) \
                -channelType    $val(chan) \
				-channelNum     $val(nc) \
                -topoInstance   $topo \
				-nodeWorkMode   0 \
				-noiseChannel   0 \
				-noiseStartTime 0 \
                -agentTrace     ON \
                -routerTrace    ON \
                -macTrace       ON \
                -movementTrace  OFF

#===================================
#        Nodes Definition        
#===================================
#Create 5 nodes
set n0 [$ns node]
$n0 set X_ 500
$n0 set Y_ 500
$n0 set Z_ 0.0
$ns initial_node_pos $n0 20
set n1 [$ns node]
$n1 set X_ 300
$n1 set Y_ 500
$n1 set Z_ 0.0
$ns initial_node_pos $n1 20
set n2 [$ns node]
$n2 set X_ 500
$n2 set Y_ 700
$n2 set Z_ 0.0
$ns initial_node_pos $n2 20
set n3 [$ns node]
$n3 set X_ 700
$n3 set Y_ 500
$n3 set Z_ 0.0
$ns initial_node_pos $n3 20
set n4 [$ns node]
$n4 set X_ 500
$n4 set Y_ 300
$n4 set Z_ 0.0
$ns initial_node_pos $n4 20
#===================================
#        PU Definition        
#===================================
#===================================
#        PU Definition        
#===================================
$ns node-config  -nodeWorkMode   2 \
                 -noiseChannel   0 \
                 -noiseStartTime 100 

set p1 [$ns node]
$god new_node $p1
$p1 set X_ 400
$p1 set Y_ 600
$p1 set Z_ 0.0
$ns initial_node_pos $p1 20

$ns node-config  -nodeWorkMode   2 \
                 -noiseChannel   1 \
                 -noiseStartTime 100 
set p2 [$ns node]
$god new_node $p2
$p2 set X_ 400
$p2 set Y_ 400
$p2 set Z_ 0.0
$ns initial_node_pos $p2 20

$ns node-config  -nodeWorkMode   2 \
                 -noiseChannel   2 \
                 -noiseStartTime 100 
set p3 [$ns node]
$god new_node $p3
$p3 set X_ 600
$p3 set Y_ 400
$p3 set Z_ 0.0
$ns initial_node_pos $p3 20

$ns node-config  -nodeWorkMode   2 \
                 -noiseChannel   3 \
                 -noiseStartTime 100 

set p4 [$ns node]
$god new_node $p4
$p4 set X_ 600
$p4 set Y_ 600
$p4 set Z_ 0.0
$ns initial_node_pos $p4 20

#===================================
# Agents Definition & Applications Definition 
#===================================
# Setup a UDP connection and Setup a CBR Application over UDP connection
set udp_1 [new Agent/UDP]
$ns attach-agent $n1 $udp_1
set null_1 [new Agent/Null]
$ns attach-agent $n0 $null_1
$ns connect $udp_1 $null_1
$udp_1 set packetSize_ 1000

set cbr_1 [new Application/Traffic/CBR]
$cbr_1 attach-agent $udp_1
$cbr_1 set packetSize_ 1000
$cbr_1 set rate_ 1.0Mb
$cbr_1 set random_ null
$ns at 1.0 "$cbr_1 start"
$ns at 250.0 "$cbr_1 stop"

set udp_2 [new Agent/UDP]
$ns attach-agent $n2 $udp_2
set null_2 [new Agent/Null]
$ns attach-agent $n0 $null_2
$ns connect $udp_2 $null_2
$udp_2 set packetSize_ 1000

set cbr_2 [new Application/Traffic/CBR]
$cbr_2 attach-agent $udp_2
$cbr_2 set packetSize_ 1000
$cbr_2 set rate_ 1.0Mb
$cbr_2 set random_ null
$ns at 1.0 "$cbr_2 start"
$ns at 250.0 "$cbr_2 stop"

set udp_3 [new Agent/UDP]
$ns attach-agent $n0 $udp_3
set null_3 [new Agent/Null]
$ns attach-agent $n3 $null_3
$ns connect $udp_3 $null_3
$udp_3 set packetSize_ 1000

set cbr_3 [new Application/Traffic/CBR]
$cbr_3 attach-agent $udp_3
$cbr_3 set packetSize_ 1000
$cbr_3 set rate_ 1.0Mb
$cbr_3 set random_ null
$ns at 1.0 "$cbr_3 start"
$ns at 250.0 "$cbr_3 stop"

set udp_4 [new Agent/UDP]
$ns attach-agent $n0 $udp_4
set null_4 [new Agent/Null]
$ns attach-agent $n4 $null_4
$ns connect $udp_4 $null_4
$udp_4 set packetSize_ 1000

set cbr_4 [new Application/Traffic/CBR]
$cbr_4 attach-agent $udp_4
$cbr_4 set packetSize_ 1000
$cbr_4 set rate_ 1.0Mb
$cbr_4 set random_ null
$ns at 1.0 "$cbr_4 start"
$ns at 250.0 "$cbr_4 stop"

#===================================
#        Termination        
#===================================
#Define a 'finish' procedure
proc finish {} {
    global ns tracefile namfile
    global val(out)
    $ns flush-trace
    close $tracefile
    close $namfile

    exec nam $val(out).nam &
    exit 0
}
for {set i 0} {$i < $val(su) } { incr i } {
    $ns at $val(stop) "\$n$i reset"
}
for {set i 1} {$i < 5 } { incr i } {
    $ns at $val(stop) "\$p$i reset"
}
$ns at $val(stop) "$ns nam-end-wireless $val(stop)"
$ns at $val(stop) "finish"
$ns at $val(stop) "puts \"done\" ; $ns halt"
$ns run
