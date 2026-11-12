MODULE_BINARY := bin/grasp-pose

$(MODULE_BINARY): Makefile go.mod *.go cmd/module/*.go
	go build -o $(MODULE_BINARY) ./cmd/module

lint:
	gofmt -s -l . | tee /dev/stderr | (! read)
	go vet ./...

test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY)
	strip $(MODULE_BINARY)
	tar czf $@ meta.json $(MODULE_BINARY)

module: test module.tar.gz

setup:
	go mod tidy
