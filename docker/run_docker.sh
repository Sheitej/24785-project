#!/bin/bash

# Remove the source file and git tracker in the host (not remove the files if it's container env)
# get th parent directory of the current directory
if [ -f /.dockerenv ]; then
  echo "It's in a Docker container. You don't remove your source file and can not start a docker container."
else
  echo "It's in host. Delete source code to clean up the folder structure in host, and start a docker container."
    parent_dir="$(dirname "$PWD")"
    for path in "$parent_dir"/*; do
        name="$(basename "$path")"
        if [[ "$name" != "docker" && "$name" != "sandbox" && "$name" != "cleanup_src_host.sh" ]]; then
            rm -rf "$path"
            echo "Delete: $path"
            # echo "Would delete: $path"
        fi
    done
    # remove git tracking files
    rm -rf "$parent_dir/.git"
    rm -rf "$parent_dir/.gitmodules"
    rm -rf "$parent_dir/.gitignore"
fi


BUILD_FLAG=${1:-no} # Default to 'no' if not specified

xhost +local:docker
source setup.env
service_name="${SETUPNAME}_${USER}"

# Overwrite the service name in docker file
sed -i "s/default_service_name/$service_name/g" docker-compose.yml

# Create a working directory, set permissions, and copy .sh file for isaac and blender
mkdir -p ../sandbox/${USER}/
chmod -R 755 ../sandbox/${USER}/

# Run docker compose
# if it gives the argument "build", it intentionally builds the docker image.
# if not it skips the building process in cases there is already an image.
if [ "$BUILD_FLAG" = "--build" ]; then 
    echo "Building Docker image."
    docker compose --env-file setup.env --project-name project_${SETUPNAME}_${USER} -f ./docker-compose.yml up --build --detach
else
    echo "Skipping build step."
    docker compose --env-file setup.env --project-name project_${SETUPNAME}_${USER} -f ./docker-compose.yml up --detach
fi

# Overwrite the service name back to the default for the next time
sed -i "s/$service_name/default_service_name/g" docker-compose.yml

# Enter the running container
docker exec -it cont_${SETUPNAME}_${USER} bash