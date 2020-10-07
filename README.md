# SA multi-Target Tracking Level and Behavior Inference Level

This repository integrates multi target tracking algorithm with graphic inference of trajectories to build the situational awareness.

## The Log of our Sessions

[Link](wiki/SessionsLogs.md)

## Dependencies

### BIL

1. Python 3 (on Ubuntu you might have to use `pip3` instead in the following commands)
2. Shapely: `pip install shapely`
3. NetworkX: `pip install networkx`
4. PyPlot: `pip install matplotlib`

## Mock Data

The data under [Mock](data/Mock/) contains the below JSON files.

* [fm.json](data/Mock/MovingSensor-0/fm.json) - the feature map - it is a graph that defines the map as a set of features, with structure to support multiple zoom levels
* [dt.json](data/Mock/MovingSensor-0/dt.json) - the delanuay triangulation - it is a list of vertices and a list of the connectivity list. Since the connectivity list is also embedded in a following variable, the key information that you would use from here is the list of vertices.
* [sam.json](data/Mock/MovingSensor-0/sam.json) - the situational awareness map - it is a graph that defines the map as a collection of triangles, but consistent with the feature map. The nodes represent the triangles (will have a one-to-one correspondence with the connectivity list of DT). The nodes also have properties - i.e. features. As I am writing this I realize that I only have names and geometric parameters in the features as of now. It is easy to add (and I will add shortly) additional properties such as occlusion.
* [obs.json](data/Mock/MovingSensor-0/obs.json) - this is the set of "observed data". I say observed, but it also includes the ground truth. This is an array corresponding to the number of objects that we have.
	* Each member has an array named `Datap` which represents the trajectory taken by this agent. Each member of this array is of form `[t, x, y, psi]` where `t` is the timestamp at the time of observation.
	* Depending on whether the agent is a team member, the object might have one of the two:
		* `FOV`: If the object is a team member then it is likely to have one or more sensors, and so correspondingly will have an array, `FOV`. Each element of the array represents one sensor. Each sensor contains two arrays: a list of the X coordinates and a list of Y coordinates.
		* `valid`: If it is not part of the team, it is considered an agent which we are observing. In that case, it contains an array `valid`. If it is 1 then it implies that the agent was observed in an FOV of a team member. Otherwise it is 0 and the data is primarily for you to figure out the ground truth.
