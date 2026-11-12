// The grasp-pose module's entrypoint.
package main

import (
	grasppose "github.com/viam-labs/grasp-pose"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	module.ModularMain(resource.APIModel{API: posetracker.API, Model: grasppose.Model})
}
