class Motor:
    def __init__(self, directionPin, pwmPin, minSpeed, maxSpeed):
        self._directionPin = directionPin
        self._pwmPin = pwmPin
        self._minSpeed = minSpeed
        self._maxSpeed = maxSpeed
        self._speed = 0
        self._direction = True

    @property
    def pwmPin(self):
        return self._pwmPin
	
    @property
    def direction(self):
	return self._direction

    @direction.setter
    def direction(self, speed):
        self._speed = speed
        if speed >= 0:
            self._direction = True
        else:
            self._direction = False

    @property
    def directionPin(self):
        return self._directionPin

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        self._speed = value

    @property
    def minSpeed(self):
        return self._minSpeed

    @minSpeed.setter
    def minSpeed(self, value):
        self._minSpeed = value

    @property
    def maxSpeed(self):
        return self._maxSpeed

    @maxSpeed.setter
    def maxSpeed(self, value):
        self._maxSpeed = value

    def stop(self):
        self._speed = 0
