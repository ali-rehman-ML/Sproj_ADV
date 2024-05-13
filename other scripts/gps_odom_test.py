from math import sin,cos,radians,degrees

def transform(lat1,lon1,lat2,lon2,heading):
    r=6371000



    tx=(lat2-lat1)*(111131)
    ty=(lon2-lon1)*(111131)

    theta=radians(heading)

    x=cos(theta)*tx +sin(theta)*ty
    y=-sin(theta)*tx + cos(theta)*ty
    print(x,y)


# import math

# def transform(lat1,lon1,lat2,lon2,heading):
#     # Convert degrees to radians
#     r=6371000

#     X0=r*cos(radians(lat1))*cos(radians(lon1))
#     Y0=r*cos(radians(lat1))*sin(radians(lon1))


#     lat_rad = math.radians(lat2)
#     long_rad = math.radians(lon2)
#     heading = math.radians(heading)


    
#     # Calculate X and Y using the first formula
#     X = r * math.cos(lat_rad) * math.cos(long_rad)
#     Y = r * math.cos(lat_rad) * math.sin(long_rad)
    
#     # Calculate X' and Y' using the rotated frame formulas
#     X_prime = ((X - X0) * math.cos(heading - math.radians(0)) - (Y - Y0) * math.sin(heading - math.radians(0)))
#     Y_prime = ((X - X0) * math.sin(heading - math.radians(0)) + (Y - Y0) * math.cos(heading - math.radians(0)))
#     print(X_prime,Y_prime)
#     return X_prime, Y_prime


# 31.472352, 74.410561#

lat_0,lon_0,lat,lon=31.4723114, 74.41049629999999, 31.4727428, 74.411023
# lat_0 = 331.472352, 74.410561#
# lon_0 = 74.410426
# lat = 31.472301  # Reference latitude (same as starting point in this case)

# lon =74.410481  
heading=56

transform(lat_0,lon_0,lat,lon,heading)
    