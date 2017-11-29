#!/usr/bin/env python

class MyClass:
    
    def __init__( self, x = 3.0, y = 2.0, name = 'Name' ):
        self.x = x
        self.y = y
        self.name = name

    def function( self ):
        return self.x + self.y
        
    def print_result( self ):
        print( '%8s - x: %5.2f  y: %5.2f  => Result: %5.2f' 
              % ( self.name, self.x, self.y, self.function() ) )


def main():
    
    a = MyClass()
    b = MyClass( 5.0, 4.0, 'B' )

    a.print_result()
    
    a.x = 6.0
    a.y = 7.0
    a.name = 'A'
    a.print_result()
    
    b.print_result()
 

if __name__ == '__main__':
    
    main()
    
