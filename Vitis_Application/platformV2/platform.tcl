# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\ECE544-EmbeddedSystemsFPGA\ECE544-ACTUAL-Project2\Vitis_Application\platformV2\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\ECE544-EmbeddedSystemsFPGA\ECE544-ACTUAL-Project2\Vitis_Application\platformV2\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {platformV2}\
-hw {C:\ECE544-EmbeddedSystemsFPGA\ECE544-ACTUAL-Project2\hdl\nexysa7fpga.xsa}\
-proc {microblaze_0} -os {freertos10_xilinx} -out {C:/ECE544-EmbeddedSystemsFPGA/ECE544-ACTUAL-Project2/Vitis_Application}

platform write
platform generate -domains 
platform active {platformV2}
platform generate
