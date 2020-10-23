function drone = rSetLed(drone,id,freq,duration)
% Use this function to launch leds animations.
% The parameter is a string containing the animation number, 
% its frequency (Hz) and its duration (s), separated with commas.
% Animation names can be found below with a short description
% following a more detailed one.

% ARDRONE_LED_ANIM_BLINK_GREEN_RED              =  0,
% ARDRONE_LED_ANIM_BLINK_GREEN                  =  1,
% ARDRONE_LED_ANIM_BLINK_RED                    =  2,
% ARDRONE_LED_ANIM_BLINK_ORANGE                 =  3,
% ARDRONE_LED_ANIM_SNAKE_GREEN_RED              =  4,
% ARDRONE_LED_ANIM_FIRE                         =  5,
% ARDRONE_LED_ANIM_STANDARD                     =  6,
% ARDRONE_LED_ANIM_RED                          =  7,
% ARDRONE_LED_ANIM_GREEN                        =  8,
% ARDRONE_LED_ANIM_RED_SNAKE                    =  9,
% ARDRONE_LED_ANIM_BLANK                        = 10,
% ARDRONE_LED_ANIM_RIGHT_MISSILE                = 11,
% ARDRONE_LED_ANIM_LEFT_MISSILE                 = 12,
% ARDRONE_LED_ANIM_DOUBLE_MISSILE               = 13,
% ARDRONE_LED_ANIM_FRONT_LEFT_GREEN_OTHERS_RED  = 14,
% ARDRONE_LED_ANIM_FRONT_RIGHT_GREEN_OTHERS_RED = 15,
% ARDRONE_LED_ANIM_REAR_RIGHT_GREEN_OTHERS_RED  = 16,
% ARDRONE_LED_ANIM_REAR_LEFT_GREEN_OTHERS_RED   = 17,
% ARDRONE_LED_ANIM_LEFT_GREEN_RIGHT_RED         = 18,
% ARDRONE_LED_ANIM_LEFT_RED_RIGHT_GREEN         = 19,
% ARDRONE_LED_ANIM_BLINK_STANDARD               = 20.
%
% 0- BLINK_GREEN_RED.
% Turn on all the four red LEDs for 500 MS, 
% then turn on all the four green LEDs for 500 MS. 
% Do it infinite if the duration argument value is 0;
% 
% 1- BLINK_GREEN.
% Wait for 500 MS then turn on all the four green LEDs for 500 MS. 
% Do it infinite if the duration argument value is 0;
% 
% 2- BLINK_RED.
% Turn on all the four red LEDs for 500 MS, 
% then turn all off and wait for 500 MS. 
% Do it infinite if the duration argument value is 0;
% 
% 3- BLINK_ORANGE.
% Turn on all the LEDs for 500 MS. The combination of red and green will give the yellow-orange color. 
% Then turn all off and wait for 500 MS. Do it infinite if the duration argument value is 0;
% 
% 4- SNAKE_GREEN_RED.
% Turn on the front left green LED and the front right red LED for 200 MS, 
% then turn on the front left red LED and the rear right green LED for 200 MS, 
% then the front right green LED and the rear right red LED for 200 MS, 
% then the front right red LED and the rear left green LED for 200 MS, 
% then the rear left red LED and the rear right green LED for 200 MS, 
% then the front left green LED and the rear right red LED for 200 MS, 
% then the front left red LED and the rear left green LED for 200 MS, 
% then the front right green LED and the rear left red LED for 200 MS. Do it infinite if the duration argument value is 0;
% 5- FIRE.
% 
% Turn on all the 2 red LED on the rear and the red and green (for yellow-orange color) LED on the front right for 50 MS, 
% then keep the 2 red LED on the rear turned on and turn on the red and green (for yellow-orange color) LED on the front left for 50 MS.
% Do it infinite if the duration argument value is 0;
% 
% 6- STANDARD.
% Turn on all the 2 green LEDs on the front and all the 2 red LEDs on the rear for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 7- RED.
% Turn on all the four red LEDs for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 8- GREEN.
% Turn on all the four green LEDs for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 9- RED_SNAKE.
% Turn on the front left red LED for 500 MS,
% then the front right red LED for 500 MS,
% then the rear right red LED for 500 MS,
% then the rear left red LED for 500 MS.
% Do it infinite if the duration argument value is 0;
% 
% 10- BLANK.
% Turn off all the LEDs from previous animations;
% 
% 11- RIGHT_MISSILE.
% Wait for 500 MS,
% then turn on the rear right red LED for 300 MS, 
% then the front right red LED and rear right red-green (orange-yellow) LEDs for 100 MS,
% then the front right green-red (orange-yellow) LEDs for 300 MS, 
% then turn all the LEDs off and wait for 500 MS.
% Do it once if the duration argument value is 0;
% 
% 12- LEFT_MISSILE.
% Wait for 500 MS,
% then turn on the rear left red LED for 300 MS, 
% then the front left red LED and rear left red-green (orange-yellow) LEDs for 100 MS,
% then the front left green-red (orange-yellow) LEDs for 300 MS, 
% then turn all the LEDs off and wait for 500 MS.
% Do it once if the duration argument value is 0;
% 
% 13- DOUBLE_MISSILE.
% Wait for 500 MS, then turn on all the red LEDs on the rear for 300 MS, 
% then all the LEDs (orange-yellow) on the front and the 2 LEDs on the front for 100 MS,
% then all the LEDs (orange-yellow) on the front for 300 MS, 
% then turn all the LEDs off and wait for 500 MS.
% Do it once if the duration argument value is 0;
% 
% 14- FRONT_LEFT_GREEN_OTHERS_RED.
% Turn on the front left green LED, front right red LED, rear left red LED and rear right red for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 15- FRONT_RIGHT_GREEN_OTHERS_RED.
% Turn on the front right green LED, front left red LED, rear left red LED and rear right red for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 16- REAR_RIGHT_GREEN_OTHERS_RED.
% Turn on the front left red LED, front right red LED, rear left red LED and rear right green for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 17- REAR_LEFT_GREEN_OTHERS_RED.
% Turn on the front left red LED, front right red LED, rear left green LED and rear right red for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 18- LEFT_GREEN_RIGHT_RED.
% Turn on the front left green LED, the front right red LED, the rear right red LED and the rear left green LED for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 19- LEFT_RED_RIGHT_GREEN.
% Turn on the front left red LED, the front right green LED, the rear right green LED and the rear left red LED for 100 MS.
% Do it once if the duration argument value is 0;
% 
% 20- BLINK_STANDARD.
% Wait for 500 MS, then turn on all the 2 front green LEDS and all the 2 rear red LEDs for 500 MS.
% Do it infinite if the duration argument value is 0;

fprintf(drone.pCom.controlChannel,sprintf('AT*CONFIG=%d,"leds:leds_anim","%d,%d,%d"\r',drone.pCom.SequenceNumber,id,ARDrone_FloatArg2Int(freq),duration)); %SetLed Mode
drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;
end

function intValue = ARDrone_FloatArg2Int(floatValue)
hex = sprintf('%tx',floatValue);
for i=1:length(hex)
    if hex(i)=='f'
        bin((i*4)-3:i*4)=[1 1 1 1];
    elseif hex(i)=='e'
        bin((i*4)-3:i*4)=[1 1 1 0];
    elseif hex(i)=='d'
        bin((i*4)-3:i*4)=[1 1 0 1];
    elseif hex(i)=='c'
        bin((i*4)-3:i*4)=[1 1 0 0];
    elseif hex(i)=='b'
        bin((i*4)-3:i*4)=[1 0 1 1];
    elseif hex(i)=='a'
        bin((i*4)-3:i*4)=[1 0 1 0];
    elseif hex(i)=='9'
        bin((i*4)-3:i*4)=[1 0 0 1];
    elseif hex(i)=='8'
        bin((i*4)-3:i*4)=[1 0 0 0];
    elseif hex(i)=='7'
        bin((i*4)-3:i*4)=[0 1 1 1];
    elseif hex(i)=='6'
        bin((i*4)-3:i*4)=[0 1 1 0];
    elseif hex(i)=='5'
        bin((i*4)-3:i*4)=[0 1 0 1];
    elseif hex(i)=='4'
        bin((i*4)-3:i*4)=[0 1 0 0];
    elseif hex(i)=='3'
        bin((i*4)-3:i*4)=[0 0 1 1];
    elseif hex(i)=='2'
        bin((i*4)-3:i*4)=[0 0 1 0];
    elseif hex(i)=='1'
        bin((i*4)-3:i*4)=[0 0 0 1];
    elseif hex(i)=='0'
        bin((i*4)-3:i*4)=[0 0 0 0];
    end
end
intValue = 0;
for i = 2: length(bin)
    intValue = intValue + bin(i)*(2^(length(bin)-i));
end
if bin(1) == 1
    intValue = intValue - 2^(length(bin)-1);
end
intValue = int32(intValue);
end