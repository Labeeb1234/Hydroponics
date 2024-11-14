## While using the amr_mani_moveit package make sure to have .setup_assitant script file populated with the script below otherwise you may run into CMAKE build errors
~~~yaml
moveit_setup_assistant_config:
  urdf:
    package: amr_mani_description
    relative_path: urdf/amr_mani.xacro
  srdf:
    relative_path: config/amr_mani.srdf
  package_settings:
    author_name: labeeb
    author_email: labeebranassar@gmail.com
    generated_timestamp: 1731477887
  control_xacro:
    command:
      - position
    state:
      - position
      - velocity
  modified_urdf:
    xacros:
      - control_xacro
  control_xacro:
    command:
      - position
    state:
      - position
      - velocity
~~~
## Note: the file name should be ".setup_assistant"
