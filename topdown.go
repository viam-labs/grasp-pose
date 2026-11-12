package grasppose

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
)

// Model is the top-down grasp model.
var Model = resource.NewModel("viam-labs", "grasp-pose", "top-down")

func init() {
	resource.RegisterComponent(posetracker.API, Model, resource.Registration[posetracker.PoseTracker, *Config]{
		Constructor: newTopDown,
	})
}

type topDown struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable
	camera   string
	settings settings
	vision   vision.Service
	vname    string
	fs       framesystem.Service
	logger   logging.Logger
}

func newTopDown(
	ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger,
) (posetracker.PoseTracker, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	vs, err := vision.FromProvider(deps, cfg.Vision)
	if err != nil {
		return nil, fmt.Errorf("vision service %q: %w", cfg.Vision, err)
	}
	fs, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, fmt.Errorf("frame system: %w", err)
	}
	return &topDown{
		Named: conf.ResourceName().AsNamed(), camera: cfg.Camera, settings: cfg.settings(),
		vision: vs, vname: cfg.Vision, fs: fs, logger: logger,
	}, nil
}

// iterable is what the vision client decodes a point cloud geometry into (a *pointcloud.BasicOctree, mm).
type iterable interface {
	Iterate(numBatches, currentBatch int, fn func(p r3.Vector, d pointcloud.Data) bool)
}

// find reads the camera's current view once and plans its grasps.
func (t *topDown) find(ctx context.Context) ([]Grasp, []Object, error) {
	detections, err := t.vision.GetDetections3D(ctx, t.camera, nil)
	if err != nil {
		return nil, nil, fmt.Errorf("vision service %q: %w", t.vname, err)
	}
	var clouds []cloud
	for _, d := range detections {
		if len(d.Transforms) == 0 {
			continue
		}
		root := d.Transforms[0]
		var points iterable
		for _, tf := range d.Transforms[1:] {
			if pc, ok := tf.Geometry().(iterable); ok {
				points = pc
				break
			}
		}
		if points == nil {
			t.logger.Debugf("skipping %s: no point cloud", root.Name())
			continue
		}
		// the root sits on the camera, which may ride an arm: pin it to the world now
		world, err := t.fs.TransformPose(ctx, referenceframe.NewPoseInFrame(root.Parent(), root.Pose()), referenceframe.World, nil)
		if err != nil {
			return nil, nil, fmt.Errorf("frame system: %w", err)
		}
		c := cloud{}
		if g := root.Geometry(); g != nil {
			c.label = g.Label()
		}
		points.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
			c.points = append(c.points, spatialmath.Compose(world.Pose(), spatialmath.NewPoseFromPoint(p)).Point())
			return true
		})
		clouds = append(clouds, c)
	}
	grasps, objects := plan(clouds, t.settings)
	return grasps, objects, nil
}

// Poses returns each grasp's world pose, named "object-<i>/grasp-<j>".
func (t *topDown) Poses(ctx context.Context, bodyNames []string, extra map[string]interface{}) (referenceframe.FrameSystemPoses, error) {
	grasps, _, err := t.find(ctx)
	if err != nil {
		return nil, err
	}
	want := map[string]bool{}
	for _, n := range bodyNames {
		want[n] = true
	}
	out := referenceframe.FrameSystemPoses{}
	for _, g := range grasps {
		if len(want) == 0 || want[g.Name] {
			out[g.Name] = referenceframe.NewPoseInFrame(referenceframe.World, g.Pose)
		}
	}
	return out, nil
}

func poseMap(p spatialmath.Pose) map[string]interface{} {
	pt, ov := p.Point(), p.Orientation().OrientationVectorDegrees()
	return map[string]interface{}{"x": pt.X, "y": pt.Y, "z": pt.Z, "o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta}
}

// DoCommand serves {"grasps": {}}: the grasps and object boxes from one read of the view.
func (t *topDown) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["grasps"]; !ok {
		return nil, fmt.Errorf(`unknown command %v: the top-down model serves {"grasps": {}}`, cmd)
	}
	grasps, objects, err := t.find(ctx)
	if err != nil {
		return nil, err
	}
	gs := []interface{}{}
	for _, g := range grasps {
		gs = append(gs, map[string]interface{}{
			"name": g.Name, "object": g.Object, "label": g.Label,
			"width_mm": g.WidthMM, "score": g.Score, "pose": poseMap(g.Pose),
		})
	}
	os := []interface{}{}
	for _, o := range objects {
		os = append(os, map[string]interface{}{
			"name": o.Name, "label": o.Label, "center": poseMap(o.Center), "yaw_deg": o.YawDeg,
			"dims_mm": map[string]interface{}{"x": o.DimsMM.X, "y": o.DimsMM.Y, "z": o.DimsMM.Z},
		})
	}
	return map[string]interface{}{"grasps": gs, "objects": os}, nil
}
