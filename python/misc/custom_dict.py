import argparse

parser = argparse.ArgumentParser(description='Generate a .yml file with a reduced dictionary size.')
parser.add_argument('numtag', metavar='numTags', type=int,
                   help='number of tags')
parser.add_argument('filename', metavar='filename.yml', type=str,
                   help='name of output YML')

args = parser.parse_args()

ids  = [0x5867,0x8b03,0x2537,0xb6c7,0xe45,0x161,0x219,0x859b,0x87,0xc93f,0x905f,0x3e73,0x6ab7,0x1baf,0x6f0f,0x23d3,0x47a5,0x8cf7,0x83cf,0x9205,0x29a5,0x8033,0x857d,0xa4af,0x422f,0x1d07,0x4ee3,0x64c5,0xaa7f,0x4b75,0x34db,0x926,0x262,0x501d,0x415,0x6201,0x2064,0x2d5,0x10b,0x9427,0xc16b,0xa603,0x911,0x1043,0x87b,0xccf,0x162b,0x9ab3,0x30b7,0xad0b,0x60a6,0x3845,0xce2b,0xadc7,0x612,0x4253,0x9cc3,0xc23,0x409b,0x8e87,0x98e5,0x20f1,0xa807,0x1bf,0x7023,0xdf1,0x2957,0x26b3,0xd80f,0x4076,0x233b,0x32e3,0x7a85,0x4349,0xc857,0x41c6,0x3813,0x6d97,0x324f,0xe3f,0x47ff,0x2217,0xdd6f,0x48b1,0x8b95,0xd9c7,0x1a7d,0x867b,0xb7ff,0x7fa7,0x478b,0x6d43,0x6167,0x8f67,0xcda7,0x5cb7,0xf3af,0x889f,0x2fdf,0xd2e7,0x553,0xbc1f,0x7607,0x2a61,0x11eb,0xa457,0x789f,0x2e95,0xb46b,0x39ff,0x4fb6,0x647f,0x250d,0x5883,0xc5d7,0xe73f,0x129f,0x548f,0xb253,0x635f,0x1ed7,0xc647,0xa1f7,0x565f,0x7e6f,0xe10f,0xf56,0xf2df,0x87a3,0x4b87,0x24f6,0xafb7,0x3bc7,0x51a7,0x6a47,0x4c46,0x6723,0x2787,0x2667,0x2f13,0x50d7,0x680d,0x7ad3,0x454f,0x6c35,0xf17f,0x6b91,0x4011,0x16,0x2022,0xa1,0x142,0x40f,0x1025,0xcb,0x45,0x4427,0x8491,0x2292,0x482b,0x8221,0x8293,0xf7,0x7d,0x601,0x84a5,0x2443,0x806f,0x21c1,0x4825,0x466,0x742d,0xca07,0x20bb,0x88a3,0x2251,0xa81,0x202d,0x205b,0x1a07,0x8119,0x6413,0x6105,0x820b,0x8715,0x1c0b,0xb003,0x307,0x147b,0x135,0x62a3,0x6c0b,0x14b3,0x5c05,0xd81,0x36d,0x8d3,0x8ba7,0xb25,0x4c5b,0x2867,0x242b,0x1237,0x4206,0xa0c3,0xa4f3,0x984b,0x8507,0x4529,0xf0d,0xb2b,0x2891,0x1a3b,0x234b,0x8d43,0x22e6,0x7245,0x1c77,0x825d,0x3487,0x60d6,0x5403,0x90e3,0xa43,0x6519,0x6169,0xc397,0x2285,0xc127,0x2c05,0x8871,0x5a0b,0x4e26,0x15af,0xf97,0x258b,0x463b,0x86c3,0x292f,0x9e3,0x2571,0x4ce5,0x81d5,0x459,0x40e2,0x5b6,0x4a33,0x837,0xa2a7,0xf4b]

maxTag = args.numtag
if(args.numtag > len(ids)):
	maxTag = len(ids)

with open(args.filename, 'w') as file:
    file.write("name CUSTOM\nnbits 16\n")
    for i in range(0, maxTag):
    	file.write(format(ids[i], '0>16b'))
    	file.write("\n")
