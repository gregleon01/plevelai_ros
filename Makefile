.PHONY: build run shell logs
build:
	docker compose build --no-cache
run:
	xhost +local:root
	docker compose up --remove-orphans
shell:
	xhost +local:root
	docker compose run --rm ros bash
logs:
	docker compose logs -f --tail=200
