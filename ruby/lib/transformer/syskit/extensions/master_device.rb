module Transformer
    # Module used to extend the device specification objects with the ability
    # to specify frames
    #
    # The #frame attribute allows to specify in which frame this device
    # produces information.
    module MasterDeviceExtension
        ## 
        # Provide transform assignments for the underlying device driver
        def use_frames(frame_mappings)
            requirements.use_frames(frame_mappings)
            self
        end
    end
end
