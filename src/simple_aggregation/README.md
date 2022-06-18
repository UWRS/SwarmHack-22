# Team 2 - Technical Solution

## Algorithm Overview
1. Random walk until obstacle detection
2. Check IR readings against pre-identified threshold to discern whether obstacle is wall or robot
3. Stop time is a constant function of obstacle type (i.e wait for shorter/longer if wall/robot encountered repectively)
3. Ant-like behaviour; eventually, cycle of aggregation and dispersion

## Notes
* MONA has 5 sensors located around its front hemisphere only

## Feedback
* Advised against dynamic stop time function, as threshold checking.
* Calibrate wheels via direct interaction with motor encoders