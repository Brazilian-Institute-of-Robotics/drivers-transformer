require 'transformer/syskit/test'

describe Transformer::TaskContextExtension do
    attr_reader :task_m

    before do
        Roby.app.import_types_from 'base'
        Roby.app.import_types_from 'transformer'
    end

    subject do
        Syskit::TaskContext.new_submodel do
            output_port 'data', '/double'
            output_port 'transform', '/base/samples/RigidBodyState'
            transformer do
                associate_frame_to_ports 'dev', 'data'
                transform_output 'transform', 'from' => 'to'
                max_latency 0.1
            end
        end.new
    end

    describe "#each_associated_port" do
        it "maps local names to global ones" do
            subject.select_frame('dev', 'test')
            assert_equal [[subject.data_port, 'test']], subject.each_annotated_port.to_a
        end
        it "yields ports for which the frame is not yet selected" do
            assert_equal [[subject.data_port, nil]], subject.each_annotated_port.to_a
        end
    end
    describe "#each_transform_port" do
        def verify_each_transform_port(expected, result)
            assert_equal expected.size, result.size
            result.each do |port, transform|
                valid = expected.any? do |exp_port, from, to|
                    exp_port == port && transform.from == from && transform.to == to
                end
                if !valid
                    flunk("unexpected #{port} #{transform.from} #{transform.to}, expected results: #{expected}")
                end
            end
        end
        it "maps local names to global ones" do
            subject.select_frame('from', 'test_from')
            subject.select_frame('to', 'test_to')
            transform_ports = subject.each_transform_port.to_a
            verify_each_transform_port([[subject.transform_port, 'test_from', 'test_to']], transform_ports)
        end
        it "yields ports for which both transform frame are not yet selected" do
            transform_ports = subject.each_transform_port.to_a
            verify_each_transform_port([[subject.transform_port, nil, nil]], transform_ports)
        end
    end
    describe "#each_transform_output" do
        it "returns nil frames if they are not selected" do
            assert_equal [[subject.transform_port, nil, nil]], subject.each_transform_output.to_a
        end
        it "returns silently if there is no transformer model" do
            flexmock(subject.model).should_receive(:transformer).once
            assert_equal [], subject.each_transform_output.to_a
        end
    end
    describe "#find_port_for_transform" do
        it "returns a suitable port if there is one" do
            assert_equal subject.transform_port, subject.find_port_for_transform('from', 'to')
        end
    end
end

