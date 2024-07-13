# Usage
1. Assure docker and docker-compose are installed on your system and that docker commands can be used without privelege escalation
2. Navigate to the container directory
3. Build the container with ```./build.sh``` 
4. Follow the below Development Usage or Producation Usage instructions depending on your use case

# Development Usage 
1. Start the container with ```./init_dev.sh```
2. Create shells running inside the container with ```./new_dev_shell.sh```
3. When shutting down use ```./stop_dev.sh```

### Important Notes:
- When started with ```./init_dev.sh``` the container will be running in idle mode until stopped with ```./stop_dev.sh```
- For changes to go into effect in development the container does not need to be rebuilt after the initial build, but to move to production, it will be
- Make sure all code that is intended to run during container operation resides in the src directory
- Make sure any system packages or python packages installed get added to ```setup/system-requirements.txt``` or ```setup/python-requirements.txt``` for those packages to be included into the container at the next build

# Production Usage
1. Start the production container with ```./init_prod.sh```