# DD/MM/YYYY
# 30/09/2020

We created the code structure.

## Questions for next meeting
* What kind of trajectory does BIL need for a good demo? (geometry)

## Assignment:
### Reza
* Create a Scenario object that takes the JSON as input
### Tianqi
* Read input data into your TTL algorithm in the run function

# 06/10/2020

Discussed data structure.

## Notes:
### Data to remove:
* fm.json: Edges
* dt.json: Constraints, ConnectivityList
* sam.json: Edges
* obs.json: valid

### Data to add:
* obs.json: id

## Assignment:
### Anant
* Generate `fm.json`, `dt.json`, `sam.json` from the FM cpp code.
* Possible consolidate the files.
### Reza
* Take JSON structure as input.
### Tianqi
* Generate `obs.json` object.

# update in 25/10/2020

Reform in `obs.json`

An example: 

```
obs.json

[

 {

    "t" = 0.1,
    "deletedTracks": [],       # track IDs deleted in previous time step
    "sensors" = [
        {"ID": 0,
        "pose": [x, y, theta],
        "FoV": [[[0, 0, 1, 1], [0, 1, 1, 0]]]}   # X, Y coordinates in 2 lists
        ...
    ],    
    "tracks" = [
        {"ID": 0,
        "pose": [x, y, theta],
        }
        ...
    ],
 }
 
 ...

]

}
```

# update in 14/11/2020

Scenario defined inside the map created by @Anant. Target tracking generated based on the
map. To run the target tracking part, please run 

```$ python3 main.py ttl (scenario number)```

2 scenarios are defined, thus the scenario number is either 1 or 2.
Data is saved in `data/obs.json` with video named `Scenario_x_Visual.mp4`.

# update in 02/01/2020

Scenario defined of the river map, tracking file named `data/obs_$scenario_num.json`, with visual of the tracking named `data/Jan22_demo_$scenario_num.json`. run the following to get it

```$ python3 main.py ttl (scenario number)```