import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D22)
# create the mcp object
mcp = MCP.MCP3008(spi, cs)
# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

# print('Raw ADC Value: ', chan0.value)
# print('ADC Voltage: ' + str(chan0.voltage) + 'V')

tolerance = 250

# keep 2 seconds worth of samples
sampling_rate = 1000
samples = [0] * (2 * sampling_rate)
time_between_samples = 1 / sampling_rate



def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # convert the left range into a 0-1 range (int)
    valueScaled = int(value - left_min) / int(left_span)

    # convert the 0-1 range into a value in the right range.
    return int(right_min + (valueScaled * right_span))

# read samples for 2 seconds
def read_input():
    for i in range(len(samples)):
        v = chan0.value
        samples[i] = v

        #print("read: " , samples[i])
        time.sleep(time_between_samples)

def identify_wave():
    maxval = max(samples)
    minval = min(samples)
    midval = (minval + (maxval - minval) / 2)
    
    num_bins = 100
    bins = [0] * num_bins
    for i in range(len(samples)):
        value = remap_range(samples[i], minval, maxval - 1, 0, num_bins - 1)
        #print("s: ", samples[i])
        #print("v: ", value)
        bins[value] += 1
    for i in range(len(bins)):
        print(i, ": ", bins[i])

    # --------------------------------------------------------------------------
    # identify square wave if most of the samples are near the minimum or the
    # maximum inputs
    bin_max = 0
    bin_min = 0
    bin_other = 0
    tolerance_square = 250
    for i in range(len(samples)):
        if (abs(maxval - samples[i]) < 6000):
            bin_max += 1
        elif (abs(samples[i] - minval) < 6000):
            bin_min += 1
        else:
            bin_other += 1
    print(bin_max)
    print(bin_min)
    print(bin_other)
    if (abs(bin_max - bin_min) < 300 and bin_other < 400):
        return "square"


    # --------------------------------------------------------------------------
    # identify triangle wave if samples are roughly evenly distributed
    tolerance_triangle = 25
    avgbincount = sum(bins) / len(bins)
    trianglewave = True
    maxdiff = 0
    for i in range(num_bins):
        if (abs(bins[i] - avgbincount) > tolerance_triangle):
            trianglewave = False
            break
        elif (abs(bins[i] - avgbincount) > maxdiff):
            maxdiff = abs(bins[i] - avgbincount)
    if (trianglewave == True):
        return "triangle"


    # --------------------------------------------------------------------------
    # identify sine wave if middle samples are roughly evenly distributed and
    # edge samples are equivalent
    tolerance_sine = 2 * tolerance

    bins_lower_quartile = 0
    bins_mid_lower_quartile = int((num_bins / 4))
    bins_mid_upper_quartile = int(2 * (num_bins / 4))
    bins_upper_quartile = int(3 * (num_bins / 4))

    # check middle 50% for even distribution
    sinewave = True
    for i in range(bins_mid_lower_quartile, bins_upper_quartile):
        if (abs(bins[i] - avgbincount) > tolerance_sine):
            sinewave = False
            break

    # check if number of samples at edges are equivalent
    if (sinewave == True):
        count_lower_quartile = 0
        count_upper_quartile = 0
        for i in range(0, bins_mid_lower_quartile):
            count_lower_quartile += bins[i]
        for i in range(bins_upper_quartile, len(bins)):
            count_upper_quartile += bins[i]

        if (abs(count_upper_quartile - count_lower_quartile) < tolerance_sine):
            return "sine"


    # --------------------------------------------------------------------------
    # no idea what the wave is
    return "unknown"

def identify_freq():
    flips = 0
    last_value = samples[0]

    maxval = max(samples)
    minval = min(samples)
    midval = (minval + maxval) / 2

    # find how many times input has crossed the midpoint
    for i in range(1, len(samples)):
        if (((last_value > midval and samples[i] < midval) or (last_value < midval and samples[i] > midval)) and abs(last_value - samples[i]) >= 1000):
            flips += 1
        last_value = samples[i]

    # correctly calculate the number of wave cycles
    cycles = flips / 4

    # calculate frequency: Number of cycles divided by the total duration
    # the total duration is the number of samples divided by the sampling rate
    if cycles > 0:
        frequency = cycles / (len(samples) / sampling_rate)
        return frequency 
    else:
        return 1



while True:
    read_input()
    wave = identify_wave()
    freq = identify_freq()
    print("Wave: ", wave)
    print("Frequency: ", freq, " Hz")
