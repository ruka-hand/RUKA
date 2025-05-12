<h1 align="center" style="font-size: 2.0em; font-weight: bold; margin-bottom: 0; border: none; border-bottom: none;">RUKA: Rethinking the Design of Humanoid Hands with Learning</h1>

##### <p align="center"> [Anya Zorin*](https://anyazorin.github.io/), [Irmak Guzey*](https://irmakguzey.github.io/), [Billy Yan](https://www.linkedin.com/in/yanyibobilly/), [Aadhithya Iyer](https://www.linkedin.com/in/aadhithya-iyer-147697176/), [Lisa Kondrich](https://www.linkedin.com/in/lisa-kondrich/), [Nikhil X. Bhattasali](https://www.hertzfoundation.org/person/nikhil-bhattasali/), [Lerrel Pinto](https://lerrelpinto.com)</p>
##### <p align="center"> New York University </p>

<p align="center">
  <img src="assets/ruka.gif">
 </p>

#####
<div align="center">
    <a href="https://ruka-hand.github.io"><img src="https://img.shields.io/static/v1?label=Project%20Page&message=Website&color=blue"></a> &ensp;
    <a href="https://arxiv.org/abs/2504.13165"><img src="https://img.shields.io/static/v1?label=Paper&message=Arxiv&color=red"></a> &ensp; 
    <a href="https://ruka.gitbook.io/instructions"><img src="https://img.shields.io/static/v1?label=Hardware&message=Instructions&color=pink"></a> &ensp;
    <a href="https://discord.gg/rMgC4cgk"><img src="https://img.shields.io/static/v1?label=Community&message=Discord&color=purple"></a> &ensp;
    <a href="https://osf.io/hwajz/"><img src="https://img.shields.io/static/v1?label=Data&message=OSF&color=orange"></a> &ensp;
    
</div>

#####


## Installation
Download the repo, create the conda environment, install requirements and install the `ruka_hand` package.
```
git clone --recurse-submodules https://github.com/ruka-hand/RUKA
cd RUKA
conda env create -f environment.yml
conda activate ruka_hand
pip install -r requirements.txt
pip install -e .
```

### Download data from OSF
All controller weights, example keypoints and data collected are stored in an OSF project, available [here](https://osf.io/hwajz/).  
You can run `./download_data.sh` to download all controller checkpoints alongside some example keypoints and controller data. This script will prompt you for an OSF username and token in order to clone the project to your local directory. A token can be generated at: https://osf.io/settings/tokens.  

After running the script, all provided data can be found un `ruka_data/osfstorage`.

Controller checkpoints will be downloaded to `ruka_data/osfstorage/checkpoints`. Within this directory, you’ll find a controller for each finger of each hand. The data structure is as follows:

```
checkpoints/
    left_index/
        config.yaml -> Was used during training 
        dataset_stats.pkl -> Stores statistics during training
        decoder_best.pkl -> Decoder weights
        encoder_best.pkl -> Encoder weights
    left_middle/ ...  
    ...
    right_index/ ...
    ...
examples/
    human_examples.npy
    robot_examples.npy
data/
    ...
```
Checkpoints are later used in `ruka_hand/control/controller.py` and is necessary for any application such as teleoperation.

## Initial Software Steps

### Connecting RUKA
Connect the RUKA hand to your workstation using a USB cable. Then, identify which port the USB is connected to. You can do this by plugging and unplugging the hand while observing the changes in the output of `ls /dev/ttyUSB*`. The port that appears when you plug in the hand corresponds to that hand (left or right).
Update the `USB_PORTS` dictionary in `ruka_hand/utils/constants.py` accordingly, e.g., `USB_PORTS = {"left": "/dev/ttyUSB0", "right": "/dev/ttyUSB1"}`.

You can try moving the motors to the reset position by running:
```
python scripts/reset_motors.py --hand_type <right|left>
```
If this moves the intended hand then it means that the initial software is completed!

**NOTE:**
If you get an error saying that 
```
serial.serialutil.SerialException: [Errno 13] could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0'
```
You might need to run following commands to add your user to `dialout` group and reboot the machine for it to take effect.
```
sudo usermod -aG dialout $USER
sudo reboot
```

### Calibrating Motor Ranges
Since RUKA is a tendon-driven hand, cable tensions can vary between different builds. To ensure consistency across builds, we provide a calibration script that automatically determines the motor ranges.  
Run the following command to save the motor limits to `RUKA/motor_limits/<left|right>_motor_limits.npy`:  
```
python calibrate_motors.py --hand-type <left|right>
```
These motor limits are later used in `ruka_hand/control/hand.py`.

If everything works well, this is how the calibration code should look like: 

<p align="center">
  <img width=300 src="assets/calibration.gif">
 </p>

During calibration, we sometimes observe that the knuckle joints don't fully curl. If you notice this behavior (for example, the index finger not fully curling as shown in the provided GIF), please gently push the finger to complete the curl.
After running the calibration, execute `python scripts/reset_motors.py --hand-type <right|left>`. This should move the fingers to a fully open position, with the tendons tensioned but the fingers remaining extended.

## Installing and Using Pre Trained Controllers

### Load and use the controllers

We provide example keypoints to run on the robot when you first build / initialize the hand. These examples are downloaded in `ruka_data/osfstorage/examples` after you run `./download_data.sh`. `robot_examples.npy` is collected on the robot and `human_examples.npy` is collected by a human using the MANUS glove. 
You can simply initialize RUKA hand with the loaded controllers with:
```
from ruka_hand.control.operator import RUKAOperator
operator = RUKAOperator(hand_type="right", moving_average_limit=2)
```
and run these examples with:
```
import os

import numpy as np

from ruka_hand.control.operator import RUKAOperator
from ruka_hand.utils.file_ops import get_repo_root
from ruka_hand.utils.timer import FrequencyTimer


def load_keypoints(test_type: str, hand_type: str):
    repo_root = get_repo_root()
    examples_dir = "ruka_data/osfstorage/examples"
    keypoints = np.load(
        os.path.join(repo_root, examples_dir, f"{test_type}_examples_{hand_type}.npy")
    )
    return keypoints


def run_controller(keypoints: np.ndarray, hand_type: str):

    timer = FrequencyTimer(10)

    # Initialize the RUKAOperator
    operator = RUKAOperator(hand_type=hand_type, moving_average_limit=2)

    # Run the controller
    for keypoint in keypoints:
        timer.start_loop()
        operator.step(keypoint)
        timer.end_loop()


if __name__ == "__main__":
    hand_type = "right"
    keypoints = load_keypoints("human", hand_type)
    run_controller(keypoints, hand_type)
```
This script will load these saved keypoints and run it on a right RUKA hand. If you built a left hand, you should change the `hand_type` parameter accordingly. This script can also be found in `examples/test_controllers.py`

These are how this script looks like on our newly built RUKA hand. 


<p align="center">
  <strong>Human</strong>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<strong>Robot</strong>
</p>

<p align="center">
  <img src="assets/human_eval.gif" width="300">
  <img src="assets/robot_eval.gif" width="300">
</p>



We recommend running this simple script before proceeding any further. If you're getting any error please make sure that you completed the previous steps, then if the issue still persists feel free to ask your questions in our [Discord Channel](https://discord.gg/rMgC4cgk) / create an issue in this repo or reach out to `irmakguzey@nyu.edu` or `az61@nyu.edu`. We have tested our framework on our setups but we are committed to help with any issue during reproducing RUKA.  


## Teleoperation

We provide scripts to teleoperate RUKAs using [Manus gloves](https://www.manus-meta.com) and an [Oculus headset](https://www.meta.com/quest/quest-3/).  
Since RUKA is also trained using MANUS data, we observe better performance with the MANUS gloves. Below are instructions for setting up both devices.

### MANUS

To run teleoperation with MANUS:

* First, update your MANUS dongle to enable Linux streaming. For more information, please visit their [website](https://www.manus-meta.com).
* Once the dongle is updated, plug it into your workstation and run the MANUS streaming client in a separate terminal using `MANUS_Core_2.4.0_SDK`, located in the `Bidex_Manus_Teleop` submodule. Use the command:  
  ```
  cd ./submodules/Bidex_Manus_Teleop/MANUS_Core_2.4.0_SDK/SDKClient_Linux
  ./SDKClient_Linux.out
  ```
  Choose the option `[1] Core Integrated - This will run standalone without the need for a MANUS Core connection`, then navigate to the Glove Menu. You should see the joint angles change as you move your fingers. **NOTE:** Here, if you get *Device Manager: Unable to open device -LSBUSB* error, make sure you have packages that are mentioned at [the MANUS SDK Instructions](https://docs.manus-meta.com/2.3.0/Plugins/SDK/Linux/Installation/) installed and try running the binary file with sudo privileges: `sudo ./SDKClient_Linux.out`. 
* Edit `HOST`, `LEFT_GLOVE_ID` and `RIGHT_GLOVE_ID` variables in `ruka_hand/utils/constants.py` accordingly to your workstation's IP address, and corresponding glove ids.
* Once the MANUS stream is running, execute the following command in a new terminal:
  ```
  python teleop.py --hand-type <left|right> --mode manus
  ```

### Oculus

* **Install the Oculus app**: We use the [OpenTeach](https://github.com/aadhithya14/Open-Teach) Oculus app to teleoperate the RUKA hand. Follow the instructions on their repository to install the VR application.
* Once the app is running, start the teleoperation using:
  ```
  python teleop.py --hand-type <left|right> --mode oculus
  ```

---

## Train Your Own Controllers

### Using Our Data

After downloading the data, edit the `all_data_directories` variable in `configs/train_controller_right_thumb.yaml` to point to your repository root:
```
all_data_directories:
  - <INSERT REPO ROOT>/ruka_data/osfstorage/data/right_thumb_1
  - <INSERT REPO ROOT>/ruka_data/osfstorage/data/right_thumb_2
```

Update the `root_dir` to specify the directory where training logs and checkpoints should be saved.

To start training, run:
```
python train_controller.py --config-name train_controller_right_thumb
```
This command will train controllers for the right thumb finger using the following architecture:

<p align="center">
    <img src="assets/architecture.png">
</p>

We also provide an example configuration for the left index finger at `configs/train_controller_left_index.yaml`. The differences between configurations include:

* `all_data_directories` – updates to reflect the appropriate data directories  
* `hand_type: <left|right>`  
* `dataset.finger_name: <Index|Thumb>`  
* `dataset.input_type: <joint_angles|fingertips>` – We use `fingertips` for the thumb and `joint_angles` for other fingers.

To train controllers with a custom architecture, you’ll need to define a new `Learner`. Refer to `ruka_hand/utils/initialize_learner.py` for examples on how learners are initialized.

### Collect your own data

If you own a MANUS glove, you can collect your own data and train your own controllers! 
* Edit `HOST`, `LEFT_GLOVE_ID` and `RIGHT_GLOVE_ID` variables in `ruka_hand/utils/constants.py` accordingly to your workstation's IP address, and corresponding glove ids.
* Then run data collection:
    ```
    python collect_data.py -ht <right|left> --demo_num <demo-num> --root_data <root-data-directory>
    ```
This will collect data using [Random Walk](https://en.wikipedia.org/wiki/Random_walk) for the thumb finger of the given hand. You can change the finger that you collect data for in line 39-44 of that script: 
```
collector.save_data_with_random_walk(
    finger_names=["Thumb"], # NOTE: Change this as you wish
    step_size=11,
    sample_num=500,
    walk_len=100,
)
```
This script will dump `ruka_data.h5` and `manus_data.h5` files to `<root-data-directory>/<hand-type>_hand/<demo-num>`. During training these files get preprocessed to `npy` files in the dataset code. 
You can preprocess them separately yourself by running: 
```
from ruka_hand.learning.preprocessor import Preprocessor

preprocessor = Preprocessor(
    save_dirs=save_dirs,
    frequency=-1,
    module_keys=["manus", "ruka"],
    visualize=False,
)

processes = preprocessor.get_processes()
for process in processes:
    process.start()

for process in processes:
    process.join()

```

## Simulation Files 
We provide `xml` files that can be used in MuJoco. They can be found under `assets` folder.

## Citing 
If you plan to use this code in your project, please consider citing our paper by adding the following entry to your bibliography file:
```
@article{zorin2025ruka,
  title={RUKA: Rethinking the Design of Humanoid Hands with Learning},
  author={Zorin, Anya and Guzey, Irmak and Yan, Billy and Iyer, Aadhithya and Kondrich, Lisa and Bhattasali, Nikhil X and Pinto, Lerrel},
  journal={arXiv preprint arXiv:2504.13165},
  year={2025}
}
```
