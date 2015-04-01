require 'transformer/test'
require 'transformer/sdf'

module Transformer
    describe SDF do
        before do
            model_dir = File.expand_path(File.join('data', 'sdf'), File.dirname(__FILE__))
            @orig_model_path = ::SDF::XML.model_path.dup
            ::SDF::XML.model_path << model_dir
        end

        after do
            ::SDF::XML.model_path = @orig_model_path
        end

        it "loads a root model just fine" do
            conf = SDF.load('model://root_model')
            assert conf.has_frame?('root_model_name')
        end

        it "prefixes frames hierarchically" do
            conf = SDF.load('model://model_within_a_world')
            assert_equal %w{world_name world_name.root_model_name},
                conf.frames.to_a.sort
        end

        it "creates a static transform between root links and the model" do
            conf = SDF.load('model://model_with_only_root_links')
            tr = conf.transformation_for('w.m.root_link', 'w.m')
            assert Eigen::Vector3.new(1, 2, 3).
                approx?(tr.translation)
            assert Eigen::Quaternion.from_angle_axis(Math::PI/2, Eigen::Vector3.UnitZ).
                approx?(tr.rotation)
        end

        it "creates dynamic transforms between root links and child links if a transformation producer is given" do
            recorder = flexmock
            recorder.should_receive(:call).with('w.m.root_link', 'w.m.child_link').once
            conf = SDF.load('model://model_with_child_links') do |parent, child|
                recorder.call(parent, child)
                'producer'
            end
            tr = conf.transformation_for('w.m.child_link', 'w.m.root_link')
            assert 'producer', tr.producer
        end
    end
end
