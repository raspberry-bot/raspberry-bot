require 'erb'

template = ERB.new File.read("#{ENV['GREENBOTS_ROOT']}/src/configs/nginx/nginx.conf.erb")
nginx_config = template.result

File.open('/etc/nginx/nginx.conf', 'w') { |file| file.write(nginx_config) }