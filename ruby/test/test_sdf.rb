require 'transformer/test'
require 'transformer/sdf'

module Transformer
    describe SDF do
        attr_reader :conf

        before do
            model_dir = File.expand_path(File.join('data', 'sdf'), File.dirname(__FILE__))
            @orig_model_path = ::SDF::XML.model_path.dup
            ::SDF::XML.model_path << model_dir
            @conf = Configuration.new
        end

        after do
            ::SDF::XML.model_path = @orig_model_path
        end

        it "loads a root model just fine" do
            conf.load_sdf('model://root_model')
            assert conf.has_frame?('root_model_name')
        end

        it "prefixes frames hierarchically" do
            conf.load_sdf('model://model_within_a_world')
            assert_equal %w{world_name world_name.root_model_name},
                conf.frames.to_a.sort
        end

        it "creates a static transform between root links and the model" do
            conf.load_sdf('model://model_with_only_root_links')
            tr = conf.transformation_for('w.m.root_link', 'w.m')
            assert Eigen::Vector3.new(1, 2, 3).
                approx?(tr.translation)
            assert Eigen::Quaternion.from_angle_axis(Math::PI/2, Eigen::Vector3.UnitZ).
                approx?(tr.rotation)
        end

        it "creates a static transform between a child link and a joint" do
            conf.load_sdf('model://model_with_child_links')
            tr = conf.transformation_for('w.m.j', 'w.m.child_link')
            assert Eigen::Vector3.new(1, 2, 3).
                approx?(tr.translation)
            assert Eigen::Quaternion.from_angle_axis(Math::PI/2, Eigen::Vector3.UnitZ).
                approx?(tr.rotation)
        end

        it "creates dynamic transforms between root links and child links if a transformation producer is given" do
            recorder = flexmock
            recorder.should_receive(:call).with('w.m.root_link', 'w.m.j').once
            conf.load_sdf('model://model_with_child_links') do |parent, joint|
                recorder.call(parent, joint)
                'producer'
            end
            tr = conf.transformation_for('w.m.j', 'w.m.root_link')
            assert 'producer', tr.producer
        end

        it "always creates example transformations between root links and child links using the axis limits" do
            conf.load_sdf('model://model_with_child_links')
            tr = conf.example_transformation_for('w.m.j', 'w.m.root_link')
            assert_equal Eigen::Vector3.Zero, tr.translation
            assert Eigen::Quaternion.from_angle_axis(Math::PI / 4, Eigen::Vector3.UnitX).approx?(tr.rotation)
        end
    end
end
