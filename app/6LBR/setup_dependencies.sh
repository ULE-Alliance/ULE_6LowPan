#/bin/bash
OBJDIR=$1
SRC_FILES=$2
C_FLAGS=$3

#echo OBJDIR $OBJDIR
mkdir $OBJDIR -p

#echo files [$SRC_FILES]

for file in $SRC_FILES ; do
	#echo "SRC $file"
	fil=`basename $file`
	#echo "Handling $fil"
	dfil=`echo $fil |sed 's/c$/d/'`
	ofil=`echo $fil |sed 's/c$/o/'`
	#echo "cut $dfil"
	echo -n "$OBJDIR/" > $OBJDIR/$dfil
	gcc -MM -MG $C_FLAGS $file >> $OBJDIR/$dfil
	#if [ $? - ne 0 ]; then
#		exit $?
#	fi
	echo "\t@gcc $C_FLAGS -c -o $OBJDIR/$ofil $file" >> $OBJDIR/$dfil
#	gcc $C_FLAGS -c -o $OBJDIR/$ofil $file
done
