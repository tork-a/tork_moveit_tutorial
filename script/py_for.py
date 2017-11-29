#!/usr/bin/env python

num_list = [ 10.0, 20.0, 30.0, 40.0, 50.0 ]

for x in num_list:
    print( x )

for i in range(5):
    print( i )
    
for n in range( len(num_list) ):
    print( "{0:2d} : {1:5.1f}".format(n, num_list[n]) )

