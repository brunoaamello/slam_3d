printf "\nSection \"InputClass\"\nIdentifier \"My Mouse\"\nMatchIsPointer \"yes\"\nOption \"AccelerationProfile\" \"-1\"\nOption \"AccelerationScheme\" \"none\"\nOption \"AccelSpeed\" \"-1\"\nEndSection\n" | sudo tee -a /usr/share/X11/xorg.conf.d/50-mouse-acceleration.conf
