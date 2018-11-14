.PHONY : default
default : ../whiskeye_robot.sdf $(DAE)
	make -s -C mesh
	make -s -C sdf
	make -s -C plugin

.PHONY : test
test : default
	cd ../test && ./launch_sim.sh

