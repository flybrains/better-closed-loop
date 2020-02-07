import hardware_parameters as hw

class Motor(object):
    def __init__(self):


    def convert_angle_for_arduino(self, inputVal, previousAngle, mult):
    	inputVal = inputVal*(256/800)
    	spr = 800
    	conv1 = spr*(1000/256)
    	newAngle1 = (inputVal*conv1)/1000
    	highLimit = 500
    	lowLimit = 300
    	midPoint = 400
    	offset = mult*spr
    	if (newAngle1 < highLimit) and (newAngle1 > lowLimit):
    		if(newAngle1 > midPoint):
    			newAngle1 = highLimit
    		else:
    			newAngle1 = lowLimit
    	newAngle1 = newAngle1 + offset
    	if np.abs(newAngle1-previousAngle) > 400:
    		if(newAngle1 > previousAngle):
    			mult = mult-1
    			offset = mult*spr
    			newAngle1 = newAngle1 - spr
    		else:
    			mult = mult + 1
    			offset = mult*spr
    			newAngle1 = newAngle1 + spr

    	previousAngle = newAngle1

    	return int(newAngle1), mult
