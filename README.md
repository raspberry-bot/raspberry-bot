# The Green Bots


## Linux image
### Steps
run dd to get image from sd card
mount it to a loop device
gparted on the loop device
unmount loop device
resized2fs on image file

### dd
bs= sets the blocksize, for example bs=1M would be 1MiB blocksize.

count= copies only this number of blocks (the default is for dd to keep going forever or until the input runs out). Ideally blocks are of bs= size but there may be incomplete reads, so if you use count= in order to copy a specific amount of data (count*bs), you should also supply iflag=fullblock.

seek= seeks this number of blocks in the output, instead of writing to the very beginning of the output device.

So, for example, this copies 1MiB worth of y\n to position 8MiB of the outputfile. So the total filesize will be 9MiB.
