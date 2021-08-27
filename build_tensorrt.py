import tensorrt as trt

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)

model_path = 'yolov5.onnx'

def contenxt_build():
    with trt.Builder(TRT_LOGGER) as builder:
        with builder.create_network(EXPLICIT_BATCH) as network:
            with trt.OnnxParser(network, TRT_LOGGER) as parser:
                with open(model_path, 'rb') as model:
                    if not parser.parse(model.read()):
                        for error in range(parser.num_errors):
                            print(parser.get_error(error))

                    with builder.create_builder_config() as config:
                        config.max_batch_size = 1
                        config.max_workspace_size = 1 << 20

                        with builder.build_engine(network, config) as engine:
                            with open('yolov5.engine', 'wb') as f:
                                f.write(engine.serialize())import tensorrt as trt


def build():
    builder = trt.Builder(TRT_LOGGER)
    config = builder.create_builder_config()

    network = builder.create_network()
    parser = trt.OnnxParser(network, TRT_LOGGER)

    with open(model_path, 'rb') as model:
        parser.parse(model.read())

    builder.max_workspace_size = 1 << 20

    with builder.build_cuda_engine(network) as engine:
        with open('yolov5.engine', 'wb') as f:
            f.write(engine.serialize())


def deserialize():
    with open('yolov5.engine', 'rb') as fo, trt.Runtime(TRT_LOGGER) as runtime:
		engine = runtime.deserialize_cuda_engine(fo.read())


if __name__ == '__main__':
    print(__name__)
