require 'transformer/syskit/test'

describe Transformer do
    describe "the handling of devices" do
        attr_reader :dev_m, :task_m
        before do
            Roby.app.import_types_from 'base'
            Roby.app.import_types_from 'transformer'
        end

        it "detects conflicts between the frame selection on the task and on the device" do
            dev_m = Syskit::Device.new_submodel do
                output_port 'trsf', 'base/samples/RigidBodyState'
            end
            task_m = Syskit::TaskContext.new_submodel do
                output_port 'transform', 'base/samples/RigidBodyState'
                driver_for dev_m, as: 'test'
                transformer do
                    transform_output 'transform', 'from' => 'to'
                    max_latency 0.1
                end
            end
            dev = robot.device(dev_m, :as => 'device').
                frame_transform('test_from' => 'test_to').
                period(0.1)

            req = task_m.
                use_frames('from' => 'other_from').
                with_arguments('test_dev' => dev)

            inhibit_fatal_messages do
                assert_raises(Transformer::FrameSelectionConflict) do
                    syskit_stub_and_deploy(req)
                end
            end
        end

        it "links the task and device frames through transform ports" do
            dev_m = Syskit::Device.new_submodel do
                output_port 'trsf', 'base/samples/RigidBodyState'
            end
            task_m = Syskit::TaskContext.new_submodel do
                output_port 'transform', 'base/samples/RigidBodyState'
                driver_for dev_m, as: 'test'
                transformer do
                    transform_output 'transform', 'from' => 'to'
                    max_latency 0.1
                end
            end
            dev = robot.device(dev_m, :as => 'device').
                frame_transform('test_from' => 'test_to').
                period(0.1)

            task = syskit_stub_and_deploy(task_m.with_arguments('test_dev' => dev))
            assert_equal Hash['from' => 'test_from', 'to' => 'test_to'],
                task.selected_frames
        end

        it "links the task and device frames through data ports" do
            dev_m = Syskit::Device.new_submodel do
                output_port 'samples', '/double'
            end
            task_m = Syskit::TaskContext.new_submodel do
                output_port 'samples', '/double'
                driver_for dev_m, as: 'test'
                transformer do
                    associate_frame_to_ports 'frame', 'samples'
                    max_latency 0.1
                end
            end
            dev = robot.device(dev_m, as: 'device').
                frame('test').period(0.1)

            task = syskit_stub_and_deploy(task_m.with_arguments('test_dev' => dev))
            assert_equal Hash['frame' => 'test'],
                task.selected_frames
        end
    end
end

