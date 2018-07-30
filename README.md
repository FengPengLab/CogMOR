# CogMOR
CogMOR-MAC simulator based on NS2.35
=============================================
1. Install ns-2.35
=============================================

You can download the ns-2.35 from the following links,

https://sourceforge.net/projects/nsnam/files/allinone/ns-allinone-2.35/ns-allinone-2.35.tar.gz/download

Install require lib for ubuntu 16.04
------------------------------------

$ sudo apt-get install build-essential autoconf automake
$ sudo apt-get install tcl8.5-dev tk8.5-dev
$ sudo apt-get install perl xgraph libxt-dev libx11-dev libxmu-dev


Install ns-2.35
===============

edit /ns-allinone-2.35/ns-2.35/linkstate/ls.h
line 137: edit 'erase' to 'this->erase'

$ ./install

(2)Copy the relevant files in the directory of ns-2.35/, 
and Paste to your installation folder of ns-allinone-2.35/ns-2.35. 

(3)Reinstallation ns-2.35

Enter folder ns-allinone-2.35/ns-2.35/
--------------------------------------
$ cd 

(2)Copy the relevant files in the directory of ns-2.35/, 
and Paste to your installation folder of ns-allinone-2.35/ns-2.35. 

(3)Reinstallation ns-2.35



Enter folder ns-allinone-2.35/ns-2.35/
--------------------------------------

$ cd ns-allinone-2.35/ns-2.35

$ make clean

$ make





After the installation process is complete. 



Validate ns-2.35 

================



You could run ./ns for testing



$ cd yourns-allinone-2.35/ns-2.35
$ make clean
$ make


After the installation process is complete. 

Validate ns-2.35 
================

You could run ./ns for testing

$ cd your
=============================================
2. Install CogMOR-MAC
=============================================

(1)Download the source code in the directory of ns-2.35/
/ns-allinone-2.35/ns-2.35
$ ./ns

If success you will see
$ %


Edit environment
================
edit ~/.bashrc

$ sudo gedit ~/.bashrc

add these line to the end of the file and
edit /path/to/ns-allinone-2.35
------------------------------
# LD_LIBRARY_PATH - 2 path
OTCL_LIB=/path/to/ns-allinone-2.35/otcl-1.14/
NS2_LIB=/path/to/ns-allinone-2.35/lib/
USR_Local_LIB=/usr/local/lib/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$OTCL_LIB:$NS2_LIB:$USR_Local_LIB

# TCL_LIBRARY - 1 path
TCL_LIB=/path/to/ns-allinone-2.35/tcl8.5.10/library/
USR_LIB=/usr/lib/
export TCL_LIBRARY=$TCL_LIBRARY:$TCL_LIB:$USR_LIB

# PATH - 6 path
XGRAPH=/path/to/ns-allinone-2.35/xgraph-12.2/:/path/to/ns-allinone-2.35/bin/:/path/to/ns-allinone-2.35/tcl8.5.10/unix/:/path/to/ns-allinone-2.35/tk8.5.10/unix/
NS=/path/to/ns-allinone-2.35/ns-2.35/
NAM=/path/to/ns-allinone-2.35/nam-1.15/
export PATH=$PATH:$XGRAPH:$NS:$NAM
----------------------------------

You should refresh .bashrc
==========================

$ source ~/.bashrc

Execute the tcl script
======================

The tcl script is in the folder CogMOR/ns-2.35/tcl_scripts/

$ ns cogorm-1.tcl 
or
$ ns cogorm-2.tcl 

You can write your own scripts

