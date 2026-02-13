from django import template

register = template.Library()


@register.simple_tag(takes_context=True)
def active_nav(context, url_name):
    """Return 'active' if the current path matches the named URL."""
    request = context.get("request")
    if request and request.resolver_match and request.resolver_match.url_name == url_name:
        return "bg-cyan-500/20 text-cyan-100 border border-cyan-500/30"
    return "text-slate-400 hover:bg-cyan-500/10 hover:text-cyan-200"
