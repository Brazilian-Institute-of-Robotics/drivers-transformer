require 'transformer/syskit/test'

describe Transformer::CompositionExtension do
    attr_reader :cmp_m, :task_m
    before do
        Roby.app.import_types_from 'base'
        @task_m = Syskit::TaskContext.new_submodel do
            output_port 'transform', '/base/samples/RigidBodyState'
            transformer do
                transform_output 'transform', 'object' => 'world'
            end
        end

        @cmp_m = Syskit::Composition.new_submodel
        cmp_m.add task_m, as: 'producer'
        cmp_m.export cmp_m.producer_child.transform_port
    end

    describe "#each_transform_output" do
        it "returns nil frames if they are not selected" do
            task = cmp_m.instanciate(plan)
            assert_equal [[task.transform_port, nil, nil]], task.each_transform_output.to_a
        end
    end
    describe "#find_port_for_transform" do
        it "returns a suitable port if there is one" do
            task = cmp_m.instanciate(plan)
            assert_equal [[task.transform_port, nil, nil]], task.each_transform_output.to_a
        end
    end
end
