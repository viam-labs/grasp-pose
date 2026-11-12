package grasppose

import (
	"context"
	"errors"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/vision/detection3d"
	"go.viam.com/test"
)

// a camera 450 mm over (300, 0), looking down and turned about its axis
var camPose = spatialmath.NewPose(r3.Vector{X: 300, Y: 0, Z: 450}, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 150})

// detection builds a detection as detection3d.FromSegmenter does: a root on the
// camera at the points' centre, carrying a box with the class label, and a
// "<root>/points" child with the points in the root's frame.
func detection(t *testing.T, name, label string, world []r3.Vector) *detection3d.Detection {
	return detectionShifted(t, name, label, world, r3.Vector{})
}

// detectionShifted puts the points link at shift under the root, the points relative to it.
func detectionShifted(t *testing.T, name, label string, world []r3.Vector, shift r3.Vector) *detection3d.Detection {
	t.Helper()
	inv := spatialmath.PoseInverse(camPose)
	cam := make([]r3.Vector, len(world))
	var centre r3.Vector
	for i, p := range world {
		cam[i] = spatialmath.Compose(inv, spatialmath.NewPoseFromPoint(p)).Point()
		centre = centre.Add(cam[i])
	}
	centre = centre.Mul(1 / float64(len(cam)))
	pc := pointcloud.NewBasicEmpty()
	for _, p := range cam {
		test.That(t, pc.Set(p.Sub(centre).Sub(shift), pointcloud.NewBasicData()), test.ShouldBeNil)
	}
	octree, err := pointcloud.ToBasicOctree(pc, 0)
	test.That(t, err, test.ShouldBeNil)
	box, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{X: 50, Y: 50, Z: 50}, label)
	test.That(t, err, test.ShouldBeNil)
	return &detection3d.Detection{Transforms: []*referenceframe.LinkInFrame{
		referenceframe.NewLinkInFrame("cam", spatialmath.NewPoseFromPoint(centre), name, box),
		referenceframe.NewLinkInFrame(name, spatialmath.NewPoseFromPoint(shift), name+"/points", octree),
	}}
}

func newTracker(t *testing.T, detect func() ([]*detection3d.Detection, error)) *topDown {
	t.Helper()
	vs := inject.NewVisionService("seg")
	vs.GetDetections3DFunc = func(ctx context.Context, camera string, extra map[string]interface{}) ([]*detection3d.Detection, error) {
		test.That(t, camera, test.ShouldEqual, "cam")
		return detect()
	}
	fs := inject.NewFrameSystemService("fs")
	fs.TransformPoseFunc = func(
		ctx context.Context, pose *referenceframe.PoseInFrame, dst string, _ []*referenceframe.LinkInFrame,
	) (*referenceframe.PoseInFrame, error) {
		test.That(t, pose.Parent(), test.ShouldEqual, "cam")
		test.That(t, dst, test.ShouldEqual, referenceframe.World)
		return referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.Compose(camPose, pose.Pose())), nil
	}
	cfg := parallelConfig()
	cfg.SupportZMM = ptr(5)
	deps := resource.Dependencies{vision.Named("seg"): vs, framesystem.PublicServiceName: fs}
	res, err := newTopDown(context.Background(), deps, resource.Config{Name: "grasps", ConvertedAttributes: cfg}, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	return res.(*topDown)
}

func oneBar(t *testing.T) func() ([]*detection3d.Detection, error) {
	return func() ([]*detection3d.Detection, error) {
		return []*detection3d.Detection{detection(t, "seg/object-0", "rectangle-red", blockPoints(320, 30, 25, 80, 40, 5, 40, true, true))}, nil
	}
}

func TestGraspsCommandReturnsWorldGraspsAndObjects(t *testing.T) {
	out, err := newTracker(t, oneBar(t)).DoCommand(context.Background(), map[string]interface{}{"grasps": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	grasps := out["grasps"].([]interface{})
	test.That(t, len(grasps), test.ShouldEqual, 1)
	g := grasps[0].(map[string]interface{})
	test.That(t, g["name"], test.ShouldEqual, "object-0/grasp-0")
	test.That(t, g["label"], test.ShouldEqual, "rectangle-red")
	pose := g["pose"].(map[string]interface{})
	test.That(t, pose["x"], test.ShouldAlmostEqual, 320, 1.5)
	test.That(t, pose["y"], test.ShouldAlmostEqual, 30, 1.5)
	test.That(t, pose["z"], test.ShouldAlmostEqual, 30, 1)
	test.That(t, pose["o_z"], test.ShouldAlmostEqual, -1, 1e-6)
	objects := out["objects"].([]interface{})
	o := objects[0].(map[string]interface{})
	test.That(t, o["name"], test.ShouldEqual, "object-0")
	test.That(t, o["yaw_deg"], test.ShouldAlmostEqual, 25, 1)
	test.That(t, o["dims_mm"].(map[string]interface{})["x"], test.ShouldAlmostEqual, 80, 3)
}

func TestPosesNamesEachGraspAndFiltersByBodyName(t *testing.T) {
	tracker := newTracker(t, oneBar(t))
	poses, err := tracker.Poses(context.Background(), nil, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(poses), test.ShouldEqual, 1)
	test.That(t, poses["object-0/grasp-0"].Parent(), test.ShouldEqual, referenceframe.World)
	poses, err = tracker.Poses(context.Background(), []string{"nope"}, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, poses, test.ShouldBeEmpty)
	poses, err = tracker.Poses(context.Background(), []string{"object-0/grasp-0"}, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(poses), test.ShouldEqual, 1)
	test.That(t, poses["object-0/grasp-0"], test.ShouldNotBeNil)
}

func TestNoDetectionsIsAnEmptyResult(t *testing.T) {
	out, err := newTracker(t, func() ([]*detection3d.Detection, error) { return nil, nil }).
		DoCommand(context.Background(), map[string]interface{}{"grasps": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["grasps"], test.ShouldBeEmpty)
	test.That(t, out["objects"], test.ShouldBeEmpty)
}

func TestAVisionErrorNamesTheService(t *testing.T) {
	_, err := newTracker(t, func() ([]*detection3d.Detection, error) { return nil, errors.New("down") }).
		Poses(context.Background(), nil, nil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `vision service "seg"`)
	test.That(t, err.Error(), test.ShouldContainSubstring, "down")
}

func TestTwoDetectionsAreNamedNearestFirst(t *testing.T) {
	out, err := newTracker(t, func() ([]*detection3d.Detection, error) {
		return []*detection3d.Detection{
			detection(t, "seg/object-0", "far", blockPoints(420, 0, 0, 40, 40, 5, 40, true, false)),
			detection(t, "seg/object-1", "near", blockPoints(250, 0, 0, 40, 40, 5, 40, true, false)),
		}, nil
	}).DoCommand(context.Background(), map[string]interface{}{"grasps": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	objects := out["objects"].([]interface{})
	test.That(t, len(objects), test.ShouldEqual, 2)
	test.That(t, objects[0].(map[string]interface{})["label"], test.ShouldEqual, "near")
	test.That(t, objects[0].(map[string]interface{})["name"], test.ShouldEqual, "object-0")
	test.That(t, objects[1].(map[string]interface{})["name"], test.ShouldEqual, "object-1")
}

func TestAFrameSystemErrorIsWrapped(t *testing.T) {
	tr := newTracker(t, oneBar(t))
	tr.fs.(*inject.FrameSystemService).TransformPoseFunc = func(
		context.Context, *referenceframe.PoseInFrame, string, []*referenceframe.LinkInFrame,
	) (*referenceframe.PoseInFrame, error) {
		return nil, errors.New("no frame cam")
	}
	_, err := tr.Poses(context.Background(), nil, nil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "frame system: no frame cam")
}

func TestAnUnknownCommandIsAnError(t *testing.T) {
	_, err := newTracker(t, oneBar(t)).DoCommand(context.Background(), map[string]interface{}{"jump": true})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "grasps")
	test.That(t, err.Error(), test.ShouldContainSubstring, "jump")
}

func TestAPointsLinkAwayFromTheRootIsComposed(t *testing.T) {
	bar := blockPoints(320, 30, 25, 80, 40, 5, 40, true, true)
	want := newTracker(t, oneBar(t))
	shifted := newTracker(t, func() ([]*detection3d.Detection, error) {
		return []*detection3d.Detection{detectionShifted(t, "seg/object-0", "rectangle-red", bar, r3.Vector{X: 10})}, nil
	})
	a, err := want.Poses(context.Background(), nil, nil)
	test.That(t, err, test.ShouldBeNil)
	b, err := shifted.Poses(context.Background(), nil, nil)
	test.That(t, err, test.ShouldBeNil)
	pa, pb := a["object-0/grasp-0"].Pose().Point(), b["object-0/grasp-0"].Pose().Point()
	test.That(t, pb.X, test.ShouldAlmostEqual, pa.X, 0.5)
	test.That(t, pb.Y, test.ShouldAlmostEqual, pa.Y, 0.5)
	test.That(t, pb.Z, test.ShouldAlmostEqual, pa.Z, 0.5)
}

func TestADetectionWithoutAPointsLinkIsSkipped(t *testing.T) {
	out, err := newTracker(t, func() ([]*detection3d.Detection, error) {
		d := detection(t, "seg/object-0", "x", blockPoints(320, 30, 25, 80, 40, 5, 40, true, true))
		d.Transforms = d.Transforms[:1]
		return []*detection3d.Detection{d}, nil
	}).DoCommand(context.Background(), map[string]interface{}{"grasps": map[string]interface{}{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["grasps"], test.ShouldBeEmpty)
	test.That(t, out["objects"], test.ShouldBeEmpty)
}
