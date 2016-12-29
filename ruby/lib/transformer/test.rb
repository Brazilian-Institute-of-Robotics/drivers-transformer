# simplecov must be loaded FIRST. Only the files required after it gets loaded
# will be profiled !!!
if ENV['TEST_ENABLE_COVERAGE'] == '1'
    begin
        require 'simplecov'
        SimpleCov.start
    rescue LoadError
        require 'transformer'
        Transformer.warn "coverage is disabled because the 'simplecov' gem cannot be loaded"
    rescue Exception => e
        require 'transformer'
        Transformer.warn "coverage is disabled: #{e.message}"
    end
end

require 'transformer'
require 'flexmock/minitest'
require 'minitest/spec'
require 'minitest/autorun'

if ENV['TEST_ENABLE_PRY'] != '0'
    begin
        require 'pry'
    rescue Exception
        Transformer.warn "debugging is disabled because the 'pry' gem cannot be loaded"
    end
end

module Transformer
    # This module is the common setup for all tests
    #
    # It should be included in the toplevel describe blocks
    #
    # @example
    #   require 'transformer/test'
    #   describe Transformer do
    #   end
    #
    module SelfTest
        def setup
            # Setup code for all the tests
        end

        def teardown
            super
            # Teardown code for all the tests
        end
    end
end

Minitest::Test.include Transformer::SelfTest

