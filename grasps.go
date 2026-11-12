package grasppose

import (
	"fmt"
	"math"
	"sort"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// cloud is one detection's points in the world frame (mm) and its class label.
type cloud struct {
	label  string
	points []r3.Vector
}

// Grasp is a pose the gripper's frame should reach, in the world frame.
type Grasp struct {
	Name, Object, Label string
	WidthMM, Score      float64
	Pose                spatialmath.Pose
}

// Object is a detection's fitted box in the world frame: x along its long side.
type Object struct {
	Name, Label string
	Center      spatialmath.Pose
	DimsMM      r3.Vector
	YawDeg      float64
}

type fitted struct {
	label       string
	points      []r3.Vector
	r           rect
	top, bottom float64
	known       bool
	id          int // index among all non-empty detections
}

// downPose points the gripper's +z straight down with its x axis along world
// heading h degrees: R = Rz(h) * Rx(pi), vision_pick.py's down(). As an
// orientation vector: (0, 0, -1) with theta 180 - h.
func downPose(x, y, z, h float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: y, Z: z}, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 180 - h})
}

// plan turns detections into grasps and object boxes (spec: Algorithm, steps 2-7).
func plan(clouds []cloud, s settings) ([]Grasp, []Object) {
	var objs []fitted
	var clusters [][]r3.Vector // every non-empty detection's own cluster, kept as an object or not
	for _, c := range clouds {
		if len(c.points) == 0 {
			continue
		}
		kept, level, known := dropSupport(c.points, s)
		pts := ownCluster(kept)
		clusters = append(clusters, pts)
		if len(pts) < minPoints {
			continue
		}
		objs = append(objs, fitted{
			label: c.label, points: pts, r: minAreaRect(pts),
			top: percentile(zs(pts), 95), bottom: level, known: known, id: len(clusters) - 1,
		})
	}
	sort.SliceStable(objs, func(i, j int) bool {
		return math.Hypot(objs[i].r.cx, objs[i].r.cy) < math.Hypot(objs[j].r.cx, objs[j].r.cy)
	})
	grasps := []Grasp{}
	objects := []Object{}
	for i, o := range objs {
		name := fmt.Sprintf("object-%d", i)
		var others []r3.Vector
		for j, pts := range clusters {
			if j != o.id {
				others = append(others, pts...)
			}
		}
		for j, g := range candidates(o, others, s) {
			g.Name, g.Object, g.Label = fmt.Sprintf("%s/grasp-%d", name, j), name, o.label
			grasps = append(grasps, g)
		}
		h := o.top - o.bottom
		objects = append(objects, Object{
			Name: name, Label: o.label, YawDeg: o.r.yawDeg,
			DimsMM: r3.Vector{X: o.r.length, Y: o.r.width, Z: h},
			Center: spatialmath.NewPose(
				r3.Vector{X: o.r.cx, Y: o.r.cy, Z: o.bottom + h/2},
				&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: o.r.yawDeg},
			),
		})
	}
	return grasps, objects
}

type candidate struct {
	grasp     Grasp
	clearance float64
}

func candidates(o fitted, others []r3.Vector, s settings) []Grasp {
	var cs []candidate
	if s.parallel {
		for _, span := range []struct{ yaw, width float64 }{{o.r.yawDeg + 90, o.r.width}, {o.r.yawDeg, o.r.length}} {
			if span.width+10 > s.maxOpening {
				continue
			}
			z := o.top - s.gripDepth
			if o.known {
				z = math.Max(z, o.bottom+s.supportClearance)
			}
			if z > o.top {
				continue
			}
			h := span.yaw
			if s.axisY {
				h -= 90
			}
			cs = append(cs, candidate{
				Grasp{WidthMM: span.width, Pose: downPose(o.r.cx, o.r.cy, z, normDeg(h))},
				fingerClearance(o.r.cx, o.r.cy, span.yaw, others, s),
			})
		}
	} else {
		var patch []r3.Vector
		for _, p := range o.points {
			if p.Z >= o.top-s.flatness {
				patch = append(patch, p)
			}
		}
		if len(patch) >= 3 {
			if pr := minAreaRect(patch); pr.width >= s.minPatch {
				cs = append(cs, candidate{
					Grasp{WidthMM: pr.width, Pose: downPose(pr.cx, pr.cy, o.top+s.cupGap, o.r.yawDeg)},
					cupClearance(pr.cx, pr.cy, others, s),
				})
			}
		}
	}
	kept := cs[:0]
	for _, c := range cs {
		if c.clearance >= 0 {
			kept = append(kept, c)
		}
	}
	sort.SliceStable(kept, func(i, j int) bool {
		if kept[i].clearance != kept[j].clearance {
			return kept[i].clearance > kept[j].clearance
		}
		return kept[i].grasp.WidthMM < kept[j].grasp.WidthMM
	})
	var out []Grasp
	for i, c := range kept {
		if i == s.graspsPerObject {
			break
		}
		c.grasp.Score = math.Min(c.clearance, 100)
		out = append(out, c.grasp)
	}
	return out
}

// boxDistance from a point to an axis-aligned box, given the point's per-axis
// distances outside the box's half extents (negative inside): negative inside.
func boxDistance(a, b float64) float64 {
	if a <= 0 && b <= 0 {
		return math.Max(a, b)
	}
	return math.Hypot(math.Max(a, 0), math.Max(b, 0))
}

// fingerClearance: top-down distance from the two fingers, fully open (the
// gripper API can't open to a width), to the nearest of the others' points.
func fingerClearance(cx, cy, spanYawDeg float64, others []r3.Vector, s settings) float64 {
	a := spanYawDeg * math.Pi / 180
	sx, sy := math.Cos(a), math.Sin(a)
	best := math.Inf(1)
	for _, side := range []float64{-1, 1} {
		off := side * (s.maxOpening/2 + s.fingerThickness/2)
		fx, fy := cx+off*sx, cy+off*sy
		for _, p := range others {
			dx, dy := p.X-fx, p.Y-fy
			along := math.Abs(dx*sx+dy*sy) - s.fingerThickness/2
			across := math.Abs(-dx*sy+dy*sx) - s.fingerWidth/2
			best = math.Min(best, boxDistance(along, across))
		}
	}
	return best
}

// cupClearance: top-down distance from the cup's circle to the nearest of the others' points.
func cupClearance(cx, cy float64, others []r3.Vector, s settings) float64 {
	best := math.Inf(1)
	for _, p := range others {
		best = math.Min(best, math.Hypot(p.X-cx, p.Y-cy)-s.cupDiameter/2)
	}
	return best
}
