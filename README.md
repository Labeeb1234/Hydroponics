#Also MOFOs don't push anything into the main branch, make your own branch --> [https://www.google.com/imgres?q=italian%20finger%20emoji&imgurl=https%3A%2F%2Fwww.clipartmax.com%2Fpng%2Fmiddle%2F286-2863171_hand-emoji-clipart-fire-emoji-italian-hand-emoji-png.png&imgrefurl=https%3A%2F%2Fwww.clipartmax.com%2Fmiddle%2Fm2H7H7i8Z5N4K9H7_hand-emoji-clipart-fire-emoji-italian-hand-emoji-png%2F&docid=VmKS9FEReOLsZM&tbnid=tJT2r4i9AmXJ4M&vet=12ahUKEwjvkZjC8duJAxXNTmwGHdvtEzQQM3oECH0QAA..i&w=840&h=749&hcb=2&ved=2ahUKEwjvkZjC8duJAxXNTmwGHdvtEzQQM3oECH0QAA]


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
