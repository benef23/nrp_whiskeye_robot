
DIR_RESOURCE=$(shell pwd | sed s!device/nrp.*!resource!)
SRC=$(DIR_RESOURCE)/180928_WhiskeyeSTL.zip
TMP=/tmp/whiskeyestl
DST=../../mesh

.PHONY : default
default : $(TMP)
	@./make_collision_mesh.py "$(TMP)" "$(DST)"

$(TMP) : $(SRC)
	unzip -d /tmp $(SRC)
	touch $(TMP)	

