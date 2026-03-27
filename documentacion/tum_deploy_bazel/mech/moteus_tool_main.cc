

#include "moteus/tool/moteus_tool.h"

#include "mjlib/io/selector.h"
#include "mjlib/multiplex/asio_client.h"

#include "mech/pi3hat_wrapper.h"

int main(int argc, char** argv) {
  boost::asio::io_context context;
  mjlib::io::Selector<mjlib::multiplex::AsioClient> client_selector{
    context.get_executor(), "client_type"};
  client_selector.Register<mjmech::mech::Pi3hatWrapper>("pi3");
  client_selector.set_default("pi3");
  return moteus::tool::moteus_tool_main(context, argc, argv, &client_selector);
}
