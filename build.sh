echo "Entering directory arduino!"
cd arduino
make clean
make depends
make -j8
make upload
