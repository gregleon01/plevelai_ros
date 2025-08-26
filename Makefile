.PHONY: build run shell logs
build:
\tdocker compose build --no-cache
run:
\txhost +local:root
\tdocker compose up --remove-orphans
shell:
\txhost +local:root
\tdocker compose run --rm ros bash
logs:
\tdocker compose logs -f --tail=200
