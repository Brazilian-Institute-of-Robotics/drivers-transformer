module Transformer
    # Module used to add the 'transforms' annotations to the graph output
    module GraphvizExtension
        def frame_transform_id(task, from, to, prefix= "")
            "frames_#{prefix}#{from}_#{to}_producer"
        end

        def add_frame_transform(task, from, to, prefix = "")
            producer = frame_transform_id(task, from, to, prefix)
            add_vertex(task, producer, "label=\"\",shape=circle")
            add_edge(["frames_#{prefix}#{from}", task], [producer, task], "dir=none")
            add_edge([producer, task], ["frames_#{prefix}#{to}", task], "")
            producer
        end

        def add_transforms_annotations
            plan.find_local_tasks(Syskit::Device).each do |device_task|
                next if !device_task.model.respond_to?(:transformer)

                tr = device_task.model.transformer
                # Add frame information stored in device definitions
                device_task.each_master_device do |dev|
                    selected_frame = dev.frame
                    selected_transform = dev.frame_transform
                    next if !selected_frame && !selected_transform

                    if selected_frame
                        add_vertex(device_task, "frames_#{dev.name}", "label=\"dev(#{dev.name})=#{selected_frame}\",shape=ellipse")
                    end
                    if selected_transform
                        from, to = selected_transform.from, selected_transform.to
                        add_vertex(device_task, "frames_dev_#{dev.name}#{from}", "label=\"dev(#{dev.name}).from=#{from}\",shape=ellipse#{",color=red" if !from}")
                        add_vertex(device_task, "frames_dev_#{dev.name}#{to}", "label=\"dev(#{dev.name}).to=#{to}\",shape=ellipse#{",color=red" if !from}")
                        transform_id = add_frame_transform(device_task, from, to, "dev_#{dev.name}")
                    end

                    device_task.find_all_driver_services_for(dev) do |srv|
                        # Two cases:
                        #  - the device is using the transformer (has frame
                        #    definitions) and declared a link between the frame
                        #    and the output port. This is handled later.
                        #  - the device is NOT using the transformer. Add the
                        #    vertex and edge now
                        srv.each_output_port do |out_port|
                            next if tr && tr.find_frame_of_port(out_port.name)

                            if transform_id && Transformer.transform_port?(out_port)
                                add_edge([transform_id, device_task], [out_port, device_task], "dir=none")
                            elsif frame_id
                                add_edge([frame_id, device_task], [out_port, device_task], "dir=none")
                            end
                        end
                    end
                end
            end

            # Add frame information stored in tasks (i.e. assigned frames)
            plan.find_local_tasks(Syskit::TaskContext).each do |task|
                tr = task.model.transformer
                next if !tr

                edges = Set.new
                tr.each_frame do |frame|
                    color = if !task.selected_frames[frame]
                                ",color=red"
                            end
                    add_vertex(task, "frames_#{frame}", "label=\"#{frame}=#{task.selected_frames[frame]}\",shape=ellipse#{color}")
                    add_edge(["inputs", task], ["frames_#{frame}", task], "style=invis")
                    add_edge(["frames_#{frame}", task], ["outputs", task], "style=invis")
                end
                tr.each_transformation do |trsf|
                    add_vertex(task, "frames_#{trsf.from}_#{trsf.to}_producer", "label=\"\",shape=circle")
                    add_edge(["frames_#{trsf.from}", task], ["frames_#{trsf.from}_#{trsf.to}_producer", task], "dir=none")
                    add_edge(["frames_#{trsf.from}_#{trsf.to}_producer", task], ["frames_#{trsf.to}", task], "")
                end
                tr.each_transform_port do |port, trsf|
                    add_edge([port, task], ["frames_#{trsf.from}_#{trsf.to}_producer", task], "dir=none,constraint=false")
                end
                tr.each_annotated_port do |port, annotated_frame_name|
                    add_edge([port, task], ["frames_#{annotated_frame_name}", task], "dir=none,constraint=false")
                end
            end
        end
    end
end
