# Checklist before making the repository public

The actual ROS package source has been incorporated. This checklist describes **remaining** release decisions, not missing source-collection steps.

- [x] Recover JetBot `cl_camera_plugins` package and final CL launch.
- [x] Recover Ubuntu VM `jetbot_mission` source and final perception/virtual scan launch.
- [x] Compare the retained launch values with the documented final configuration.
- [x] Verify Python parse and ROS XML syntax.
- [x] Exclude model weights, raw data, lab maps, screenshots, `.git` directories and development backups from the source-only archive.
- [ ] Verify the JetBot C++ plugin builds (`catkin_make -j2`) and the VM package launches after install.
- [ ] Archive/describe the **modified** `jetbot_pro` YAML files that the launch relies on, or clearly leave them as site-specific prerequisites.
- [ ] Decide whether the Caffe `deploy.prototxt` can be redistributed; confirm its original source/licence.
- [ ] Confirm permission to publish code developed in the hosting lab (and any supervisor/employer IP policy).
- [ ] Agree on a **project-wide licence**. `jetbot_mission/package.xml` is `TODO`; the C++ package declares BSD-3-Clause without an accompanying licence file. Avoid assuming the licence applies to all files.
- [ ] Review the source for any environment-specific or confidential information before public release.
- [ ] Review any later-added model files, training images, maps, logs, reports or screenshots for third-party rights, privacy and lab-layout disclosures.
- [ ] If you claim the project can be reproduced from scratch, validate the steps on a clean ROS 1 Melodic installation.

This is a non-certified experimental system. Do not claim proven safe obstacle avoidance, collision-free navigation or a measured success rate.
