
OUTFILE=__init__.py
rm -f $OUTFILE

while [ "$1" != "" ];
do
	echo "from ._$1 import *" >> $OUTFILE
	shift
done

